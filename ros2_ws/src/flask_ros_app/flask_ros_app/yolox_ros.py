#!/usr/bin/env python
import argparse
from importlib import import_module
import os
import threading

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
from HitbotInterface import HitbotInterface
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
#from .yolox.data.data_augment import ValTransform
#from .yolox.data.datasets import COCO_CLASSES
#from .yolox.exp import get_exp
#from .yolox.utils import fuse_model, get_model_info, postprocess, setup_logger, vis

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header,String
from cv_bridge import CvBridge,CvBridgeError
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data
from bboxes_ex_msgs.msg import BoundingBoxes
from bboxes_ex_msgs.msg import BoundingBox
from bboxes_ex_msgs.msg import BoundingBoxesCords
from .yolox_ros_py_utils.utils import yolox_py
from sensor_msgs.msg import CameraInfo



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


broker="172.27.34.62"
redis_server="172.27.34.62"
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
#mqtt_client = Mqtt(app)
y = 0


hi=HitbotInterface(92); #//92 is robotid
hi.net_port_initial()
ret=hi.initial(1,210); #
print(hi.is_connect())
print(hi.unlock_position())
hi.get_scara_param()
hi.wait_stop()
ret=hi.movel_xyz(hi.x,hi.y+10,-20,55,20)
print("init set,return:{}".format(ret))
hi.wait_stop()
hi.get_scara_param()
hi.wait_stop()
r.set("global_camera_xy",str(hi.x)+","+str(hi.y))
print(r.get("global_camera_xy"))

bounding_boxes_cords=None
class yolox_ros(yolox_py):
    def __init__(self) -> None:
        raw_image_topic = '/camera/color/image_rect_raw'
        depth_image_topic = '/camera/depth/image_rect_raw'
        depth_info_topic = '/camera/depth/camera_info'


        # ROS2 init
        super().__init__('yolox_ros', load_params=False)

        #self.setting_yolox_exp()
        self.bridge = CvBridge()
        
        self.imshow_isshow=False
        self.img_rgb=np.zeros( (512,512,3), dtype=np.uint8);#cv2.imread('/home/jimmy/Downloads/mushroomproject/ros2_ws/src/flask_ros_app/flask_ros_app/mushroom.jpg', 0)
        self.pub_boxes_img = self.create_publisher(Image,"/yolox/boxes_image", 10)
        self.sub_boxes = self.create_subscription(BoundingBoxesCords, "/yolox/bounding_boxes_cords", self.boxes_cords_callback, 1)
        self.gripper_publisher = self.create_publisher(String, '/yolox/gripper_hold', 1)
        self.gripper_open_pub = self.create_publisher(String, '/yolox/gripper_open', 1)
        
        self.intrinsics = None
        self.pix = None
        self.pix_grade = None
        self.scan_i=0;
        self.scan_j=0;
        self.count=0;
        self.pre_count=-1;

        #if (self.sensor_qos_mode):
        #    self.sub = self.create_subscription(Image,"/yolox/boxes_image",self.imageflow_callback, qos_profile_sensor_data)
        #else:
        self.sub = self.create_subscription(Image,"/yolox/boxes_image",self.imageflow_callback, 10)

    
    def imageflow_callback(self,msg:Image) -> None:
            global bounding_boxes
            self.img_rgb =self.bridge.imgmsg_to_cv2(msg,"bgr8")
            hi.get_scara_param()
            rett=0
            hi.wait_stop()
            #r.set("mode","camera_stop")
            logger.info(" jimmy scan current location :{},{},{},".format(hi.x,hi.y,hi.z))
            #self.pre_count=self.pre_count+1;
            #if self.count==self.pre_count:
            #    r.set("scan","start");

            if r.get("scan")=="start":
                self.pre_count=self.count;
                self.count=self.count+1;
                r.set("mode","pic_ready");
                if self.scan_i<3 and self.scan_j==3:
                    self.scan_i=self.scan_i+1;
                    self.scan_j=0;
                    rett=hi.movel_xyz(hi.x+70,hi.y,hi.z,55,80)
                    hi.wait_stop()
                    

                if self.scan_j<3:
                    self.scan_j=self.scan_j+1;
                    if self.scan_i%2==1:
                        hi.get_scara_param()
                        hi.wait_stop()
                        rett=hi.movel_xyz(hi.x,hi.y-70,hi.z,55,80)
                        hi.wait_stop()
                    else:
                        hi.get_scara_param()
                        hi.wait_stop()
                        rett=hi.movel_xyz(hi.x,hi.y+70,hi.z,55,80)
                        hi.wait_stop()

                r.set("mode","camera_ready");
                time.sleep(2)
                logger.info("scan  x rett={}:i={},j={},".format(rett,self.scan_i,self.scan_j))
                if self.scan_i==3 or rett>1 :
                    self.scan_j=0;
                    self.scan_i=0;
                    r.set("scan","stop");
                    r.set("mode","camera_ready");
                    if 1:#rett>1:
                        rett=hi.movel_xyz(300,50,hi.z,55,80)
                        hi.wait_stop()   
                        time.sleep(1) 
            hi.get_scara_param()
            hi.wait_stop()
            r.set("global_camera_xy",str(hi.x)+","+str(hi.y))

            try:

                if (self.imshow_isshow):
                    #cv2.imshow("YOLOX",img_rgb)
                    cv2.waitKey(1)
            except Exception as e:
                logger.error(e)
                pass
    def boxes_cords_callback(self, data):
        global bounding_boxes_cords
        bounding_boxes_cords=data.bounding_boxes
        logger.info(data.bounding_boxes)
        hi.get_scara_param()
        hi.wait_stop()
        #if 1:#r.get("mode")=="camera_ready":
        #    bounding_boxes_cords=data.bounding_boxes
            #r.set("mode","pickup_ready")
        r.set("mode","pickup_ready")
        logger.info(r.llen("queue"))  
        logger.info("stop?,{}".format(r.get("scan")=="stop"))  
        if r.get("scan")=="stop" and r.hlen("detections")>0 : #r.get("scan")=="stop" and
            #ele=r.lpop("queue") 
            #v=r.hget("detections",ele)
            for key in r.hkeys("detections"):
                v=r.hget("detections",key)
                xy=[]
                if v is not None:
                    xy=v.split(",")
                    logger.info(xy)
                if v is not None and len(xy)>0:
                    rett=hi.movel_xyz(int(xy[0]),int(xy[1]),-200,55,40)
                    logger.info("rett:{}".format(rett))
                    hi.wait_stop()
                    back_z=hi.z
                    hi.get_scara_param()
                    hi.wait_stop()
                    logger.info("movedown current location :{},{},{},".format(xy[0],xy[1],hi.z))
                    if rett==1:
                        r.hdel("detections",key)
                        gripper_msg = String()
                        gripper_msg.data = '%d,%d,%d' %(int(xy[0]),int(xy[1]),hi.z) 
                        self.gripper_publisher.publish(gripper_msg)
                    logger.info(r.get("mode")=="catch_over")
                    #while  r.get("mode")!="catch_over":
                    #    time.sleep(1)
                    #time.sleep(1)
                    #logger.info("current location :{},{},{},".format(xy[0],xy[1],hi.z))
                    rett=hi.movel_xyz(int(xy[0]),int(xy[1]),0,55,80)
                    hi.wait_stop()
                    hi.get_scara_param()
                    hi.wait_stop()
                    logger.info("moveup  current location :{},{},{},".format(xy[0],xy[1],hi.z))
                    gripper_msg2 = String()
                    gripper_msg2.data = 'gripper open' 
                    self.gripper_open_pub.publish(gripper_msg2)
                    time.sleep(2)
            
                hi.get_scara_param()
                hi.wait_stop()
                r.set("global_camera_xy",str(hi.x)+","+str(hi.y))
        r.set("mode","camera_ready")
        r.set("scan","start")
        img_rgb_pub = self.bridge.cv2_to_imgmsg(self.img_rgb,"bgr8")
        self.pub_boxes_img.publish(img_rgb_pub)


class web_ros(Node):
    def __init__(self) -> None:
        super().__init__('web_ros')
        app.run(host='0.0.0.0', threaded=True,port='5001')
    def web_app():
        app.run(host='0.0.0.0', threaded=True,port='5001')
    

    
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

    #rclpy.init(args=args)
    #yolox_ros_node = yolox_ros()
    #web_ros_node = web_ros()
    #executor = rclpy.executors.MultiThreadedExecutor()
    #executor.add_node(yolox_ros_node)
    #executor.add_node(web_ros_node)
    #executor_thread=threading.Thread(target=executor.spin, daemon=True)
    #executor_thread.start()
    #executor_thread.join()
    #rclpy.shutdown()


if __name__ == '__main__':
    ros_main()


