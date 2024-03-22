#!/usr/bin/env python
import argparse
from importlib import import_module
import os

import redis
from flask import Flask, render_template, Response, jsonify

from threading import Event
import requests
from flask import Flask, render_template, request, jsonify
from flask_mqtt import Mqtt
# from flask_socketio import SocketIO
import redis
import serial
import datetime
import time
import os
from time import sleep
import math
from flask_cors import CORS, cross_origin
import RPi.GPIO as GPIO

import time
from yolox_ros_py.HitbotInterface import HitbotInterface
import redis
from paho.mqtt import client as mqtt_client

import numpy as np
import cv2
from numpy import empty
from loguru import logger
from yolox_ros_py.camera_ipcam import Predictor

from numpy import empty
import torch
import torch.backends.cudnn as cudnn
from yolox_ros_py.yolox.data.data_augment import ValTransform
from yolox_ros_py.yolox.data.datasets import COCO_CLASSES
from yolox_ros_py.yolox.exp import get_exp
from yolox_ros_py.yolox.utils import fuse_model, get_model_info, postprocess, setup_logger, vis

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from cv_bridge import CvBridge,CvBridgeError
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data
from bboxes_ex_msgs.msg import BoundingBoxes
from bboxes_ex_msgs.msg import BoundingBox
from yolox_ros_py.yolox_ros_py_utils.utils import yolox_py
from sensor_msgs.msg import CameraInfo
broker=''
redis_server=''
try:
    for line in open("../ip.txt"):
        if line[0:6] == "broker":
            broker = line[9:len(line)-1]
        if line[0:6] == "reddis":
            redis_server=line[9:len(line)-1]
except:
    pass
print(broker+" "+redis_server)

pool = redis.ConnectionPool(host=redis_server, port=6379,password='jimmy')
r = redis.Redis(connection_pool=pool)

hi=HitbotInterface(92); #//92 is robotid
hi.net_port_initial()
ret=hi.initial(1,210); #
print(hi.is_connect())
print(hi.unlock_position())
hi.get_scara_param()
logger.info("hi :{},{},{}".format(hi.x,hi.y,hi.z))
rett=hi.movel_xyz(hi.x-30,0,0,25,20)
logger.info("rett:{}".format(rett))
hi.wait_stop()
r.set("global_camera_xy",str(hi.x)+","+str(hi.y))

print(r.get("global_camera_xy"))
# import camera driver
if os.environ.get('CAMERA'):
    Camera = import_module('camera_' + os.environ['CAMERA']).Camera
else:
    from .camera_ipcam import Camera

import pyrealsense2 as rs2
if (not hasattr(rs2, 'intrinsics')):
    import pyrealsense2.pyrealsense2 as rs2

i = 0



redis_server = ''
broker = ''
try:
    for line in open("../ip.txt"):
        if line[0:6] == "broker":
            broker = line[9:len(line) - 1]
        if line[0:6] == "reddis":
            redis_server = line[9:len(line) - 1]
except:
    pass
#broker = broker.replace("\n", "").replace("\r\n", "")
print(broker)
print(redis_server)
pool = redis.ConnectionPool(host=redis_server, port=6379, decode_responses=True, password='jimmy')
r = redis.Redis(connection_pool=pool)

app = Flask(__name__)
app.config['DEBUG'] = False
app.config['MQTT_BROKER_URL'] = broker
app.config['MQTT_BROKER_PORT'] = 1883
app.config['MQTT_USERNAME'] = ''  # Set this item when you need to verify username and password
app.config['MQTT_PASSWORD'] = ''  # Set this item when you need to verify username and password
app.config['MQTT_KEEPALIVE'] = 5  # Set KeepAlive time in seconds
app.config['MQTT_TLS_ENABLED'] = False  # If your server supports TLS, set it True
cors = CORS(app)
app.config['CORS_HEADERS'] = 'Content-Type'

topic = '/flask/scan'
topic2 = '/flask/xyz'
topic3 = '/flask/serial'
topic4 = '/flask/pickup'
topic5 = '/flask/home'
topic6 = '/flask/drop'
topic7 = '/flask/stop'
mqtt_client = Mqtt(app)
y = 0





@app.route('/move/<string:direction>')
def move(direction,amount=0,custom_x=0,custom_y=0):
    global camera_x,camera_y,_movement_step,_feedback,hi
    if direction == "home":
        hi.movej_angle(0,0,0,25, 100, 0)
        hi.wait_stop()
        hi.get_scara_param()
        r.set("global_camera_xy",str(hi.x)+","+str(hi.y))
        return "home"

    elif direction == "custom":
        x = float(request.args.get('x', 0))
        y = float(request.args.get('y', 0))
        z = float(request.args.get('z', 0))
        r = float(request.args.get('r', 20))
        roughly = float(request.args.get('roughly', 0))
        lr = int(request.args.get('lr', 1))
        hi.get_scara_param()
        rett=hi.movel_xyz(hi.x+x,hi.y+y,hi.z+z,r,100)
        #custom?x=0&y=0&z=-10&r=0&roughly=0
        #res = hi.new_movej_xyz_lr(0,0,-100,0,100,0,1)
        hi.wait_stop()
        ret=f"ret={rett}: x = {hi.x+x} y = {hi.y+y} z = {hi.z+z} roughly = {roughly} "
        resp={'ret':ret}
        hi.get_scara_param()
        r.set("global_camera_xy",str(hi.x)+","+str(hi.y))
        return jsonify(resp)

    else:
        if "amount" in request.args:
            amount = float(request.args.get('amount', _movement_step))
        x=0
        y=0
        z=0

        if direction=="right":
            x=-amount
        elif direction=="left":
            x=amount
        elif direction=="up":
            y=amount
        elif direction=="down":
            y=-amount
        elif direction=="top":
            z=amount
        elif direction=="bottom":
            z=-amount
        hi.new_movej_xyz_lr(x,y,z,20,100,0,1)
        hi.wait_stop()
        ret=f"{direction} x = {x}y = {y}z = {z}"
        resp={'ret':ret}
        hi.get_scara_param()
        r.set("global_camera_xy",str(hi.x)+","+str(hi.y))
        return jsonify(resp)



@app.route('/scan')
def scan():
    global prev_value
    r.set("mode", "camera_ready")
    r.delete("detections")
          #while 1:
        #rett=hi.movel_xyz(hi.x,hi.y+50,hi.z,25,20)
        #hi.wait_stop()
    #    rett=hi.movel_xyz(hi.x,hi.y+50,hi.z,25,20)
    #    hi.wait_stop()
    #hi.get_scara_param()
        #r.set("global_camera_xy",str(hi.x)+","+str(hi.y))
    #r.set("mode", "camera_ready")
           #time.sleep(100)
       #rett=hi.movel_xyz(hi.x+50,hi.y-100,hi.z,25,20)
    return " camera_ready OK";


@app.route('/xbackward')
def xbackward():

    time.sleep(1)
    xy = r.get("global_camera_xy").split(",")
    print(xy)
    r.set("global_camera_xy", str(int(xy[0]) - 25) + "," + xy[1])
    return render_template('index.html');


@app.route('/xforward')
def xfarward():
    return render_template('index.html');




@app.route('/catch')
def catch():
    #video_dir.move_increase_y(10)
    print("catch")
    return render_template('index.html');


@app.route('/release')
def release():
    #video_dir.move_decrease_y(10)
    return render_template('index.html');
@app.route('/zup')
def zup():
    #video_dir.move_decrease_x(10)
    return render_template('index.html');
@app.route('/zdown')
def zdown():
    #video_dir.move_increase_x(10)
    return render_template('index.html');





def autopick():
    publish_result = mqtt_client.publish(topic, "/flask/scan")
    return render_template('index.html');


@app.route('/zerosetting')
def zero():
    # publish_result = mqtt_client.publish(topic, "/flask/home")
    time.sleep(1)
    r.set("global_camera_xy", "0,0")
    return render_template('index.html');


@app.route('/home')
def home():
    # publish_result = mqtt_client.publish(topic, "/flask/home")
    time.sleep(1)
    r.set("global_camera_xy", "0,0")
    return render_template('index.html');



@app.route('/')
def index():
    return render_template('index.html')


@app.route('/msg')
def msg():
    return render_template('index.html')


@app.route('/update_mushroom_map' ,methods=['GET'])
def update_mushroom_map():
    detections=r.hgetall("detections")
    print(jsonify({"response":detections}))
    return jsonify({"response":detections})

def gen(camera):
    """Video streaming generator function."""
    yield b'--frame\r\n'
    while True:
        frame = camera.get_frame()
        yield b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n--frame\r\n'


@app.route('/video_feed')
def video_feed():
    """Video streaming route. Put this in the src attribute of an img tag."""
    return Response(gen(Camera()),
                    mimetype='multipart/x-mixed-replace; boundary=--frame')



@mqtt_client.on_connect()
def handle_connect(client, userdata, flags, rc):
    if rc == 0:
        #print('App_ipcam topic3 =/flask/serial Connected successfully')
        mqtt_client.subscribe(topic3)  # subscribe topic
    else:
        print('Bad connection. Code:', rc)


@mqtt_client.on_message()
def handle_mqtt_message(client, userdata, message):
    # print(message.topic)
    data = dict(topic=message.topic, payload=message.payload.decode())
    #print(' message on topic: {topic} with payload: {payload}'.format(**data))
    xyz = data['payload']
    if message.topic == topic3:
        r.set("mode","pickup_ready")
        print("get xy payload=" + xyz)
        xyz = data['payload']
        real_xyz = xyz.split(",")

        # detections=r.hgetall("detections")
        # for item in r.hkeys("detections"):
        #    real_xyz=r.hget("detections",item).split(",")
        # real_xyz=detections[0].split(",")
        real_x = int(float(real_xyz[0]) * 1000)
        real_y = int(float(real_xyz[1]) * 1000)
        real_z = int(float(real_xyz[2]) * 1000)
        #real_y = real_y - 190
        #real_x = real_x + 10
        x = real_x
        y =-real_y
        #z=-real_z
        if (abs(x) > 10 or abs(y) > 10):
            track_id=real_xyz[2]
            hi.get_scara_param()
            cam_x=hi.x
            cam_y=hi.y
            r.set("global_camera_xy",str(cam_x)+","+str(cam_y))
            #new_camera_x=x+cam_x
            #new_camera_y=y+cam_y
            #print("new_camera_xy:",new_camera_x,new_camera_y)
            #distance = int(math.sqrt(new_camera_x * new_camera_x + new_camera_y * new_camera_y))
            #print(distance)
            #print("distance:", distance)
            #detected_index=r.zrangebyscore("detections_index",min=distance-50,max=distance +50)
            #detected=r.zrangebyscore("detections_index",min=track_id,max=track_id)
            #print("len(detected):" ,len(detected))
            all=r.hgetall("detections")
            print("all:" ,all)

            #if len(detected_index)<1:
            if len(all)>0:
                new_camera_x=0;
                new_camera_y=0;
                track_id=0;
                items=all.items()
                for k,v in items:
                    print(k,v)
                    detection_xyz=v.split(",");
                    new_camera_x=float(detection_xyz[0]);
                    new_camera_y=float(detection_xyz[1]);
                    track_id=detection_xyz[2];
                    break

                #obj=str(new_camera_x) + "," + str(new_camera_y) +"," + str(track_id)
                #r.zadd("detections_index",{obj:distance} )
                #r.hset("detections", str(distance), str(new_camera_x) + "," + str(new_camera_y) +"," + str(track_id))
                print("move to new_camera:",new_camera_x,new_camera_y)
                rett=hi.movel_xyz(new_camera_x,new_camera_y,hi.z,25,20)
                hi.wait_stop()
                if rett>0:
                    r.hdel("detections",track_id)
                hi.get_scara_param()
                r.set("global_camera_xy",str(hi.x)+","+str(hi.y))
                print(rett)
                #rett=hi.movel_xyz(50,0,hi.z,25,20)
                #hi.wait_stop()
                #rett=hi.movel_xyz(new_camera_x,new_camera_y,hi.z,25,20)
                #hi.wait_stop()
                time.sleep(1)
        r.set("mode","camera_ready")



bounding_boxes=None
class yolox_ros(yolox_py):
    def __init__(self) -> None:
        raw_image_topic = '/camera/camera/color/image_rect_raw'
        depth_image_topic = '/camera/camera/depth/image_rect_raw'
        depth_info_topic = '/camera/camera/depth/camera_info'


        # ROS2 init
        super().__init__('yolox_ros', load_params=False)

        self.setting_yolox_exp()


        
        self.bridge = CvBridge()
        
        self.pub = self.create_publisher(BoundingBoxes,"/yolox/bounding_boxes", 10)
        self.sub_depth_image = self.create_subscription(Image, depth_image_topic, self.imageDepthCallback, 1)
        self.sub_info = self.create_subscription(CameraInfo, depth_info_topic, self.imageDepthInfoCallback, 1)
        self.sub_boxes = self.create_subscription(BoundingBoxes, "/yolox/bounding_boxes", self.boxes_callback, 1)

        self.intrinsics = None
        self.pix = None
        self.pix_grade = None

        if (self.sensor_qos_mode):
            self.sub = self.create_subscription(Image,"/camera/camera/color/image_rect_raw",self.imageflow_callback, qos_profile_sensor_data)
        else:
            self.sub = self.create_subscription(Image,"/camera/camera/color/image_rect_raw",self.imageflow_callback, 10)

    
    def setting_yolox_exp(self) -> None:

        WEIGHTS_PATH = '../../weights/yolox_nano.pth'  #for no trt


        self.declare_parameter('imshow_isshow',True)
        self.declare_parameter('yolox_exp_py', '/home/jimmy/Downloads/mushroomproject/ros2_ws/src/YOLOX-ROS/yolox_ros_py/exps/yolox_nano.py')
        #self.declare_parameter('yolox_exp_py', 'yolox_vos_s.py')
        self.declare_parameter('fuse',False)
        self.declare_parameter('trt', True)
        self.declare_parameter('fp16', False)
        self.declare_parameter('legacy', False)
        self.declare_parameter('device', "gpu")
        # self.declare_parameter('', 0)
        self.declare_parameter('ckpt', WEIGHTS_PATH)
        self.declare_parameter('conf', 0.3)

        # nmsthre -> threshold
        self.declare_parameter('threshold', 0.55)
        # --tsize -> resize
        self.declare_parameter('resize', 640)
        
        self.declare_parameter('sensor_qos_mode', False)

        # =============================================================
        self.imshow_isshow = self.get_parameter('imshow_isshow').value

        exp_py = self.get_parameter('yolox_exp_py').value

        fuse = self.get_parameter('fuse').value
        trt = True#self.get_parameter('trt').value
        fp16 = self.get_parameter('fp16').value
        device = self.get_parameter('device').value

        ckpt = self.get_parameter('ckpt').value
        conf = self.get_parameter('conf').value
        legacy = self.get_parameter('legacy').value
        threshold = self.get_parameter('threshold').value
        
        input_shape_w = self.get_parameter('resize').value
        input_shape_h = input_shape_w

        self.sensor_qos_mode = self.get_parameter('sensor_qos_mode').value

        # ==============================================================

        cudnn.benchmark = True
        exp = get_exp(exp_py, None)

        #BASE_PATH = os.getcwd()
        #file_name = os.path.join(BASE_PATH, "../YOLOX-main/YOLOX_outputs/yolox_voc_s/")
        file_name = "/home/jimmy/Downloads/mushroomproject/YOLOX-main/YOLOX_outputs/yolox_voc_s/"#ros2_ws/src/YOLOX-ROS/weights/tensorrt/"#os.path.join(BASE_PATH, "/src/YOLOX-ROS/weights/tensorrt/") #
        # os.makedirs(file_name, exist_ok=True)

        exp.test_conf = conf # test conf
        exp.threshold = threshold # nms threshold
        exp.test_size = (input_shape_h, input_shape_w) # test size

        model = exp.get_model()
        logger.info("Model Summary: {}".format(get_model_info(model, exp.test_size)))

        if device == "gpu":
            model.cuda()
            #if fp16:
            #    model.half() 
        # torch.cuda.set_device()
        # model.cuda()
        model.eval()

        # about not trt
        if not trt:
            if ckpt is None:
                ckpt_file = os.path.join(file_name, "best_ckpt.pth")
            else:
                ckpt_file = ckpt
            logger.info("loading checkpoint")
            ckpt = torch.load(ckpt_file, map_location="cpu")
            # load the model state dict
            model.load_state_dict(ckpt["model"])
            logger.info("loaded checkpoint done.")

        # about fuse
        if fuse:
            logger.info("\tFusing model...")
            model = fuse_model(model)

        # TensorRT
        logger.info("trt: {},file_name:{}".format(trt,file_name))
        if trt:
            assert not fuse, "TensorRT model is not support model fusing!"
            trt_file = os.path.join(file_name, "model_trt.pth") 
            logger.info("trt_file:{}".format(trt_file))
            assert os.path.exists(
                trt_file
            ), "TensorRT model is not found!\n Run python3 tools/trt.py first!"
            model.head.decode_in_inference = False
            decoder = model.head.decode_outputs
            logger.info("Using TensorRT to inference")
        else:
            trt_file = None
            decoder = None


        self.predictor = Predictor(model, exp, COCO_CLASSES, trt_file, decoder, device, fp16, legacy)


    def imageflow_callback(self,msg:Image) -> None:
            global bounding_boxes
            img_rgb = self.bridge.imgmsg_to_cv2(msg,"bgr8")
            outputs, img_info = self.predictor.inference(img_rgb)
            #logger.info("outputs: {},".format(outputs))

            try:
                logger.info(r.get("mode")=="camera_ready")
                if r.get("mode")=="camera_ready":
                    result_img_rgb, bboxes, scores, cls, cls_names,track_ids = self.predictor.visual(outputs[0], img_info)
                    bboxes_msg = self.yolox2bboxes_msgs(bboxes, scores, cls, cls_names,track_ids, msg.header, img_rgb)
                    self.pub.publish(bboxes_msg) #bboxes_msg
                else:
                    result_img_rgb=img_rgb

                #if (self.imshow_isshow):
                #    cv2.imshow("YOLOX",result_img_rgb)
                #    cv2.waitKey(1)
            except Exception as e:
                logger.error(e)
                pass
    def imageDepthCallback(self, data):
        global bounding_boxes
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
            # pick one pixel among all the pixels with the closest range:
            #indices = np.array(np.where(cv_image == cv_image[cv_image > 0].min()))[:,0]

            if bounding_boxes is not None:
                r.set("mode","pickup_ready")
                print(bounding_boxes)
                hi.get_scara_param()
                hi.wait_stop()
                camera_x=hi.x
                camera_y=hi.y
                camera_z=hi.z
                for box in bounding_boxes:
                    #logger.info("probability,%4.2f,%s,x=%4.2f,y=%4.2f",box.probability,box.class_id,(box.xmin+box.xmax)/2,(box.ymin+box.ymax)/2)
                    line ='probability:%4.2f,track_id:%s'%(box.probability,box.class_id)
                    #pix = (indices[1], indices[0])
                    pix = (int((box.xmin+box.xmax)/2),int((box.ymin+box.ymax)/2))
                    self.pix = pix
                    #line += '\tDepth at pixel(%3d, %3d): %7.1f(mm).' % (pix[0], pix[1], cv_image[pix[1], pix[0]])
                    if self.intrinsics:
                        depth = cv_image[pix[1], pix[0]]
                        result = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [pix[0], pix[1]], depth)
                        line += '  Coordinate: %8.2f %8.2f %8.2f.' % (result[0], result[1], result[2])
                    if (not self.pix_grade is None):
                        line += ' Grade: %2d' % self.pix_grade
                    line += '\r'
                    logger.info(line)
                    obj=str(int(result[0]))+","+str(int(result[1]))+","+str(int(result[2]))
                    if not r.hexists("detections",str(box.class_id)):
                        r.hset("detections", box.class_id, obj)
                        r.hset("detections_history", box.class_id, obj)                        
                        rett=hi.movel_xyz(camera_x+int(result[0]),camera_y-int(result[1]),camera_z,25,20)
                        logger.info("rett:{}".format(rett))
                        hi.wait_stop()
                        time.sleep(2)
                    else:
                         r.hdel("detections", box.class_id)


                    #rett=hi.movel_xyz(30,0,hi.z,65,20)
                    #logger.info("origin rett:{}".format(rett))
                    #hi.wait_stop()


                r.set("mode","camera_ready")
                bounding_boxes=None
        except CvBridgeError as e:
            print(e)
            return
        except ValueError as e:
            return
    
    def imageDepthInfoCallback(self, cameraInfo):
        try:
            if self.intrinsics:
                return
            self.intrinsics = rs2.intrinsics()
            self.intrinsics.width = cameraInfo.width
            self.intrinsics.height = cameraInfo.height
            self.intrinsics.ppx = cameraInfo.k[2]
            self.intrinsics.ppy = cameraInfo.k[5]
            self.intrinsics.fx = cameraInfo.k[0]
            self.intrinsics.fy = cameraInfo.k[4]
            if cameraInfo.distortion_model == 'plumb_bob':
                self.intrinsics.model = rs2.distortion.brown_conrady
            elif cameraInfo.distortion_model == 'equidistant':
                self.intrinsics.model = rs2.distortion.kannala_brandt4
            self.intrinsics.coeffs = [i for i in cameraInfo.d]
        except CvBridgeError as e:
            print(e)
            return
    def boxes_callback(self, data):
        global bounding_boxes
        bounding_boxes=None
        if 1:#r.get("mode")=="camera_ready":
            bounding_boxes=data.bounding_boxes
            #r.set("mode","pickup_ready")
        #for box in bounding_boxes:
        #    logger.info(" boxes_callback probability,%4.2f,%s,x=%4.2f,y=%4.2f",box.probability,box.class_id,(box.xmin+box.xmax)/2,(box.ymin+box.ymax)/2)


def ros_main(args = None):
            rclpy.init(args=args)
            ros_class = yolox_ros()

            try:
                rclpy.spin(ros_class)
            except KeyboardInterrupt:
                pass
            finally:
                ros_class.destroy_node()
                rclpy.shutdown()

if __name__ == '__main__':
    ros_main()
    app.run(host='0.0.0.0', threaded=True,port='5001')

