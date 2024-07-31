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
#from yolox_ros_py.HitbotInterface import HitbotInterface
import redis
from paho.mqtt import client as mqtt_client

import numpy as np
import cv2
from numpy import empty
from loguru import logger
from yolox_ros_py.camera_ipcam import Predictor

import pyrealsense2 as pyrs
from numpy import empty
import torch
import torch.backends.cudnn as cudnn
from .yolox.data.data_augment import ValTransform
from .yolox.data.datasets import COCO_CLASSES
from .yolox.exp import get_exp
from .yolox.utils import fuse_model, get_model_info, postprocess, setup_logger, vis

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from cv_bridge import CvBridge,CvBridgeError
from sensor_msgs.msg import Image,PointCloud2
from rclpy.qos import qos_profile_sensor_data
from bboxes_ex_msgs.msg import BoundingBoxes,BoundingBoxesCords
from bboxes_ex_msgs.msg import BoundingBox,BoundingBoxCord
from .yolox_ros_py_utils.utils import yolox_py
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String
#import orbslam3

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


# import camera driver
if os.environ.get('CAMERA'):
    Camera = import_module('camera_' + os.environ['CAMERA']).Camera
else:
    from .camera_ipcam import Camera

import pyrealsense2 as rs2
if (not hasattr(rs2, 'intrinsics')):
    import pyrealsense2.pyrealsense2 as rs2

i = 0



#broker="172.27.34.65"
redis_server="172.27.34.70"


pool = redis.ConnectionPool(host=redis_server, port=6379, decode_responses=True, password='jimmy')
r = redis.Redis(connection_pool=pool)

#--vocab_file=third_party/ORB_SLAM3/Vocabulary/ORBvoc.txt \
#    --settings_file=third_party/ORB_SLAM3/Examples/RGB-D/TUM1.yaml \
#parser = argparse.ArgumentParser()
#parser.add_argument("--vocab_file", required=True)
#parser.add_argument("--settings_file", required=True)
#parser.add_argument("--dataset_path", required=True)
#args = parser.parse_args()

#img_files = sorted(glob(os.path.join(args.dataset_path, 'rgb/*.png')))
#vocab_file="/media/jimmy/01D9E5979B0C5780/ORB-SLAM3-python/third_party/ORB_SLAM3/Vocabulary/ORBvoc.txt"
#settings_file="/media/jimmy/01D9E5979B0C5780/ORB-SLAM3-python/third_party/ORB_SLAM3/Examples/RGB-D/TUM1.yaml"
#slam = orbslam3.system(vocab_file, settings_file, orbslam3.Sensor.RGBD)
#slam.set_use_viewer(True)
#slam.initialize()


app = Flask(__name__)
app.config['DEBUG'] = False

cors = CORS(app)
app.config['CORS_HEADERS'] = 'Content-Type'


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







#bounding_boxes=None
bboxes_msg=None
result_img_rgb=None
img_rgb=None
i=0
class yolox_ros(yolox_py):
    def __init__(self) -> None:
        raw_image_topic = '/camera/color/image_rect_raw'
        depth_image_topic = '/camera/depth/image_rect_raw'
        depth_info_topic = '/camera/depth/camera_info'
        move_x="/move/x"

        # ROS2 init
        super().__init__('yolox_ros', load_params=False)

        self.setting_yolox_exp()


        self.bridge = CvBridge()
        
        #self.pub = self.create_publisher(BoundingBoxes,"/yolox/bounding_boxes", 10)
        self.pub_bounding_boxes_cords = self.create_publisher(BoundingBoxesCords,"/yolox/bounding_boxes_cords", 1)
        self.pub_boxes_img = self.create_publisher(Image,"/yolox/boxes_image", 10)
        self.pub_pointclouds = self.create_publisher(PointCloud2,'/yolox/pointclouds', 10)
        self.sub_depth_image = self.create_subscription(Image, depth_image_topic, self.imageDepthCallback, 1)
        self.sub_info = self.create_subscription(CameraInfo, depth_info_topic, self.imageDepthInfoCallback, 1)
        _info = self.create_subscription(String, move_x, self.MoveXYZCallback, 1)
        self.intrinsics = None
        self.pix = None
        self.pix_grade = None

        if (self.sensor_qos_mode):
            self.sub = self.create_subscription(Image,raw_image_topic,self.imageflow_callback, qos_profile_sensor_data)
        else:
            self.sub = self.create_subscription(Image,raw_image_topic,self.imageflow_callback, 10)

    
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

        # add for orbslam3
        #self.declare_parameter('--vocab_file', vocab_file)


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
            global bboxes_msg,result_img_rgb,img_rgb
            img_rgb = self.bridge.imgmsg_to_cv2(msg,"bgr8")
            outputs, img_info = self.predictor.inference(img_rgb)
            #logger.info("outputs: {},".format(outputs))

            try:
                logger.info(r.get("mode")=="camera_ready")
                if r.get("mode")=="camera_ready":
                    result_img_rgb, bboxes, scores, cls, cls_names,track_ids = self.predictor.visual(outputs[0], img_info)
                    bboxes_msg = self.yolox2bboxes_msgs(bboxes, scores, cls, cls_names,track_ids, msg.header, img_rgb)

                    #self.pub.publish(bboxes_msg) #bboxes_msg
                else:
                    result_img_rgb=img_rgb
                img_rgb_result = self.bridge.cv2_to_imgmsg(result_img_rgb,"bgr8")

                self.pub_boxes_img.publish(img_rgb_result)

                #if (self.imshow_isshow):
                #    cv2.imshow("YOLOX",result_img_rgb)
                #    cv2.waitKey(1)
            except Exception as e:
                logger.error(e)
                pass
    def depth2PointCloud(self,depth, rgb,box):
        intrinsics = self.intrinsics
        depth = np.asanyarray(depth)  # * depth_scale # 1000 mm => 0.001 meters
        rgb = np.asanyarray(rgb)
        #rows,cols  = depth.shape
        rows=480#self.intrinsics.width 
        cols=848#self.intrinsics.height
        logger.info("depth:{}".format(depth.shape))

        c, r = np.meshgrid(np.arange(cols), np.arange(rows), sparse=True)
        r = r.astype(float)
        c = c.astype(float)
        logger.info("r:{},c:{}".format(r,c))


        logger.info("box.xmin:{},box.xmax:{},box.ymin:{},box.ymax:{}".format(box.xmin,box.xmax,box.ymin,box.ymax))
        valid = (depth > 0) #& (depth < clip_distance_max) #remove from the depth image all values above a given value (meters).
        valid = np.ravel(valid)
        z = depth[box.ymin:box.ymax,box.xmin:box.xmax]
        #z = depth[box.xmin:box.xmax,box.ymin:box.ymax] 
        logger.info("z:{}".format(z))
        x =  z * (c[:,box.xmin:box.xmax] - intrinsics.ppx) / intrinsics.fx
        y =  z * (r[box.ymin:box.ymax,:] - intrinsics.ppy) / intrinsics.fy
        logger.info("x,y:{},{}".format(x,y))
        z = np.ravel(z)[valid]
        x = np.ravel(x)[valid]
        y = np.ravel(y)[valid]
        
        r = np.ravel(rgb[box.ymin:box.ymax,box.xmin:box.xmax,0])[valid]
        g = np.ravel(rgb[box.ymin:box.ymax,box.xmin:box.xmax,1])[valid]
        b = np.ravel(rgb[box.ymin:box.ymax,box.xmin:box.xmax,2])[valid]
        
        pointsxyzrgb = np.dstack((x, y, z, r, g, b))
        pointsxyzrgb = pointsxyzrgb.reshape(-1,6)
        #pointsxyzrgb=pointsxyzrgb[pointsxyzrgb]
        #logger.info("points_xyz_rgb:{}".format(pointsxyzrgb.shape))
        return pointsxyzrgb
    def create_point_cloud_file2(self,vertices, filename):
        ply_header = '''ply
    format ascii 1.0
    element vertex %(vert_num)d
    property float x
    property float y
    property float z
    property uchar red
    property uchar green
    property uchar blue
    end_header
    '''
        with open(filename, 'w') as f:
            f.write(ply_header %dict(vert_num=len(vertices)))
            np.savetxt(f,vertices,'%f %f %f %d %d %d')

    def imageDepthCallback(self, data):
        global bboxes_msg,result_img_rgb,i

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
  
            # pick one pixel among all the pixels with the closest range:
            #indices = np.array(np.where(cv_image == cv_image[cv_image > 0].min()))[:,0]
            global_camera_xy=r.get("global_camera_xy")
            camera_xy=global_camera_xy.split(",")
            logger.info("camera_xy:{}, {}".format(camera_xy[0],camera_xy[1]))

            if bboxes_msg is not None and len(bboxes_msg.bounding_boxes)>0 and r.get("mode")=="camera_ready" and  self.intrinsics:

                
                #r.set("mode","pickup_ready")


                boxes_cords=BoundingBoxesCords()

                for box in bboxes_msg.bounding_boxes:
                    if(result_img_rgb is not None):
                        points_xyz_rgb = self.depth2PointCloud(cv_image, result_img_rgb,box)
                        self.create_point_cloud_file2(points_xyz_rgb,"room{}.ply".format(i))
                        i=i+1


                    box_cord=BoundingBoxCord()
                    logger.info("probability,{},pixal x={},y={}".format(box.probability,box.class_id,(box.xmin+box.xmax)/2,(box.ymin+box.ymax)/2))
                    line ='probability:%4.2f,track_id:%s'%(box.probability,box.class_id)
                    #pix = (indices[1], indices[0])
                    pix = (int((box.xmin+box.xmax)/2),int((box.ymin+box.ymax)/2))
                    self.pix = pix
                    #line += '\tDepth at pixel(%3d, %3d): %7.1f(mm).' % (pix[0], pix[1], cv_image[pix[1], pix[0]])
                    if self.intrinsics:
                        depth = cv_image[pix[1], pix[0]]
                        result = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [pix[0], pix[1]], depth)
                        line += '  local Coordinate: %8.2f %8.2f %8.2f.' % (result[0], result[1], result[2])
                    if (not self.pix_grade is None):
                        line += ' Grade: %2d' % self.pix_grade
                    line += '\r'

                    #logger.info("detections id:{},if exist {}".format(box.class_id,r.hexists("detections",str(box.class_id))))
                    #x=(int(float(result[0])))            #Arm#-------> Y    #camera   <--- #Y
                    #y=-(int(float(result[1])))               #|              #|
                    #y-=170                                  #\|/      X      #\|/         X               
                     
                    #x=(int(float(result[0])))            #Arm#-------> Y    #camera   --- >#Y
                    #y=-(int(float(result[1])))               #|              #/|\
                    #y-=270                                 #\|/      X       #|         X   
                    # 
                    x=(int(float(result[1])))           #Arm#-------> Y    #camera   --- >#Y
                    y=(int(float(result[0])))               #|              #   \
                    y-=150                                 #\|/      X       #  \|/        X                   
                    obj=str(int(float(camera_xy[0]))+x)+","+str(int(float(camera_xy[1]))+y)+","+str(int(float(result[2])))
                    logger.info(line)
                    if not r.hexists("detections",str(box.class_id)):
                        r.hset("detections", box.class_id, obj)
                        r.lpush("queue",box.class_id)
                        r.hset("detections_history", box.class_id, obj) 
                        box_cord.x=int(float(camera_xy[0]))+x
                        box_cord.y=int(float(camera_xy[1]))+y
                        box_cord.class_id= box.class_id                      
                        boxes_cords.bounding_boxes.append(box_cord)
                    #else:
                    #     r.hdel("detections", box.class_id)
                pcl_msg = PointCloud2()
                pcl_msg.data = np.ndarray.tobytes(points_xyz_rgb)
                pcl_msg.header=Header()
                pcl_msg.header.stamp = self.get_clock().now().to_msg()#seconds #rospy.Time(t_us/10000000.0)
                pcl_msg.header.frame_id = "/map"
                self.pub_pointclouds.publish(pcl_msg)
                logger.info("pcl_msg:{}".format(pcl_msg))

                logger.info(boxes_cords)
                self.pub_bounding_boxes_cords.publish(boxes_cords)
                bboxes_msg=None
                #pose =slam.process_image_rgbd(result_img_rgb,cv_image,data.header.stamp)
                #logger(pose)
        except CvBridgeError as e:
            print(e)
            return
        except ValueError as e:
            return
        
    def MoveXYZCallback(self, data):
        global bboxes_msg
        logger.info("/move/x {}".format(data.data))
        xyz=data.data.split(";")
        x=xyz[0]
        y=xyz[1]
        z=xyz[2]
        logger.info(" local cord  to camera:{}, {}".format(x,y))
        try:

            global_camera_xy=r.get("global_camera_xy")
            camera_xy=global_camera_xy.split(",")
            logger.info("camera_xy:{}, {}".format(camera_xy[0],camera_xy[1]))
            if r.get("mode")=="camera_ready":
                #r.set("mode","pickup_ready")                            
                    obj=str(int(float(camera_xy[0]))+x)+","+str(int(float(camera_xy[1]))+y)+","+str(int(float(z)))
                    logger.info(line)
                    if 1:
                        r.hset("detections", "-1", obj)
                        r.lpush("queue","-1")
                        r.hset("detections_history", "-1", obj) 
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
        logger.info("boxes_callback")
        #global bounding_boxes
        #bounding_boxes=None
        #if 1:#r.get("mode")=="camera_ready":
         #   bounding_boxes=data.bounding_boxes
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

