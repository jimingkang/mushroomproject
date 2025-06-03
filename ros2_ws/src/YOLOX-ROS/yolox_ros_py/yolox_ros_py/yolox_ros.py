#!/usr/bin/env python
import argparse
from importlib import import_module
import os

import redis
from flask import Flask, render_template, Response, jsonify
import signal
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

#import ros_numpy
#from ros_numpy import numpy_msg
import ros2_numpy as rnp

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

from rclpy.qos import QoSProfile
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from message_filters import Subscriber, ApproximateTimeSynchronizer
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from cv_bridge import CvBridge,CvBridgeError
from sensor_msgs.msg import Image,PointCloud2,PointField
from rclpy.qos import qos_profile_sensor_data
from bboxes_ex_msgs.msg import BoundingBoxes,BoundingBoxesCords
from bboxes_ex_msgs.msg import BoundingBox,BoundingBoxCord
from .yolox_ros_py_utils.utils import yolox_py
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String
#import orbslam3
#from slam import Mapp,Frame
from scipy.spatial.transform import Rotation as R
# camera intrinsics
W, H = 848-12, 480+10
F = 430.79
K = np.array([[F,0,W//2],[0,F,H//2],[0,0,1]])
#mapp = Mapp(W, H)
frame=None



# import camera driver
if os.environ.get('CAMERA'):
    Camera = import_module('camera_' + os.environ['CAMERA']).Camera
else:
    from .camera_ipcam import Camera

import pyrealsense2 as rs2
if (not hasattr(rs2, 'intrinsics')):
    import pyrealsense2.pyrealsense2 as rs2

i = 0



broker="172.27.34.62"
redis_server="localhost"


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

ros_class=None


def gen(camera):
    """Video streaming generator function."""
    yield b'--frame\r\n'
    while True:
        frame = camera.get_frame()
        yield b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n--frame\r\n'


#@app.route('/video_feed')
#def video_feed():
#    """Video streaming route. Put this in the src attribute of an img tag."""
#    return Response(gen(Camera()),
#                    mimetype='multipart/x-mixed-replace; boundary=--frame')



@app.route('/video_feed')
def video_feed():
    global ros_class
    return Response(ros_class.gen(Camera()),mimetype='multipart/x-mixed-replace; boundary=frame')







#bounding_boxes=None
bboxes_msg=None
result_img_rgb=None
img_rgb=None
points_xyz_rgb=np.asarray([[]])
pointsxyzrgb_total=np.asarray([[0,0,0,0,0,0]])
global_points_xyz_rgb_list=np.asarray([()])
i=0
class yolox_ros(yolox_py):
    def __init__(self) -> None:
        raw_image_topic = '/camera/color/image_raw'
        depth_image_topic = '/camera/aligned_depth_to_color/image_raw' #  '/camera/depth/image_rect_raw' # /camera/aligned_depth_to_color/image_raw
        depth_info_topic = '/camera/depth/camera_info'
        move_x="/move/x"

        # ROS2 init
        super().__init__('yolox_ros', load_params=False)

        self.setting_yolox_exp()
        self.bridge = CvBridge()
        self.pub_bounding_boxes = self.create_publisher(String,"/yolox/bounding_boxes", 1)
        self.pub_bounding_boxes_cords = self.create_publisher(BoundingBoxesCords,"/yolox/bounding_boxes_cords", 1)
        self.pub_boxes_img = self.create_publisher(Image,"/yolox/boxes_image", 10)
        self.pub_pointclouds = self.create_publisher(PointCloud2,'/yolox/pointclouds', 10)
       
        self.sub_info = self.create_subscription(CameraInfo, depth_info_topic, self.imageDepthInfoCallback, 1)
        self.sub_move_xy_info = self.create_subscription(String, move_x, self.MoveXYZCallback, 1)


        self.intrinsics = None
        self.pix = None
        self.pix_grade = None
        self.pre_mushroom=[0,0,0]
        self.pre_pix=(0,0)



        qos_profile_subscriber = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,history=HistoryPolicy.KEEP_LAST,depth=1)
        self.sub_depth_image = Subscriber(self,Image, depth_image_topic)
        #if (self.sensor_qos_mode):
        self.sub = Subscriber(self,Image,raw_image_topic, qos_profile=qos_profile_subscriber)
        #else:
        #    self.sub = Subscriber(self,Image,raw_image_topic, 10)

        #self.sub = self.create_subscription(self,Image,raw_image_topic,qos_profile_sensor_data)

        ats = ApproximateTimeSynchronizer([ self.sub,self.sub_depth_image], queue_size=1, slop=0.025, allow_headerless=True)
        ats.registerCallback(self.callback)
  
    def setting_yolox_exp(self) -> None:

        WEIGHTS_PATH = '../../weights/yolox_nano.pth'  #for no trt
        self.declare_parameter('imshow_isshow',True)
        self.declare_parameter('yolox_exp_py', '/home/a/Downloads/mushroomproject/ros2_ws/src/YOLOX-ROS/yolox_ros_py/exps/yolox_nano.py')
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
        file_name = "/home/a/Downloads/mushroomproject/YOLOX-main/YOLOX_outputs/yolox_voc_s/"#ros2_ws/src/YOLOX-ROS/weights/tensorrt/"#os.path.join(BASE_PATH, "/src/YOLOX-ROS/weights/tensorrt/") #
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



    def callback(self,msg:Image,data:Image) -> None:
        global bboxes_msg,result_img_rgb,img_rgb,mapp,frame
        bboxes=[]
        scores=[]
        img_rgb = self.bridge.imgmsg_to_cv2(msg,"bgr8")
        if img_rgb is not None :
            outputs, img_info = self.predictor.inference(img_rgb)
            try:
                logger.info("mode={},mode==camera_ready,{}".format(r.get("mode"),r.get("mode")=="camera_ready"))
                if  (outputs is not None): #and r.get("mode")=="camera_ready":# and r.get("scan")=="start" :#
                    result_img_rgb, bboxes, scores, cls, cls_names,track_ids = self.predictor.visual(outputs[0], img_info)

                if result_img_rgb is not None:
                    img_rgb_pub = self.bridge.cv2_to_imgmsg(result_img_rgb,"bgr8")
                else:
                    img_rgb_pub = self.bridge.cv2_to_imgmsg(img_rgb,"bgr8")
                self.pub_boxes_img.publish(img_rgb_pub)

        #depth image
                cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
                global_camera_xy=r.get("global_camera_xy")
                camera_xy=global_camera_xy.split(",")
                logger.info("in  imageDepthCallback camera_xy:{},bboxes:{},scores:{},data.encoding{}".format(camera_xy,bboxes,scores,data.encoding))

                for box,score in zip(bboxes,scores):
                    logger.info("box[0]:{},{},{},{}".format(box[0],box[2],box[1],box[3]))                
                
                    pix = (int((box[0]+box[2])/2),int((box[1]+box[3])/2))
                    #self.diff_pix=abs(self.pre_pix[0]-pix[0])+abs(self.pre_pix[1]-pix[1])
                    logger.info("pix: {},box:{},score:{}".format(pix,box,score))
                    if( score>0.3 and self.intrinsics and abs(int((box[0]-box[2])))>10 and  abs(int((box[0]-box[2])))<120 and abs(int((box[1]-box[3])))>10 and abs(int((box[1]-box[3])))<120):
                        depth = cv_image[pix[1], pix[0]]
                        logger.info("before  rs2_deproject_pixel_to_point depth:{}".format(depth))
                        result = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [pix[0], pix[1]], depth)
                        logger.info("after  rs2_deproject_pixel_to_point depth:{}".format(depth))
                        #depth_val = cv_image.get_distance(pix[0], pix[1])  # Meters
                        #xyz = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [pix[0], pix[1]], depth_val)
                        line = f'{result[0]},{result[1]},{result[2]}'
                        logger.info("xyz in side camera: {}".format(line))
                        bbox=String()
                        bbox.data=line

                        #diff=abs(self.pre_mushroom[0]-result[0])+abs(self.pre_mushroom[2]-result[2])
                        #logger.info("box:{},".format(bbox.data))
                        if   r.get("mode")=="camera_ready": # int(result[1])<100 and
                            self.pre_mushroom=result
                            self.pre_pix=pix
                            self.pub_bounding_boxes.publish(bbox)
                        break


            except Exception as e:
                logger.error(e)
                pass

        
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
                    #logger.info(line)
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


def ros2_thread(node):
    print('entering ros2 thread')
    rclpy.spin(node)
    print('leaving ros2 thread')


def sigint_handler(signal, frame):
    """
    SIGINT handler
    We have to know when to tell rclpy to shut down, because
    it's in a child thread which would stall the main thread
    shutdown sequence. So we use this handler to call
    rclpy.shutdown() and then call the previously-installed
    SIGINT handler for Flask
    """
    rclpy.shutdown()
    if prev_sigint_handler is not None:
        prev_sigint_handler(signal)


#rclpy.init(args=None)
#ros2_node = MovePublisher()
#app = Flask(__name__)
#threading.Thread(target=ros2_thread, args=[ros2_node]).start()
#prev_sigint_handler = signal.signal(signal.SIGINT, sigint_handler)

def ros_main(args = None):
        rclpy.init(args=args)
        ros_class = yolox_ros()
        ros_class.create_rate(2)

        try:
            rclpy.spin(ros_class)
        except KeyboardInterrupt:
            pass
        finally:
            ros_class.destroy_node()
            rclpy.shutdown()
             
    #rclpy.init(args=None)
    #ros_class = yolox_ros()
    ##app = Flask(__name__)
    #threading.Thread(target=ros2_thread, args=[ros_class]).start()
    #prev_sigint_handler = signal.signal(signal.SIGINT, sigint_handler)

if __name__ == '__main__':
    ros_main()
    app.run(host='0.0.0.0', threaded=True,port='5001')
    logger.info("after app run()")


