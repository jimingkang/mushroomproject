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
from .HitbotInterface import HitbotInterface
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
from .yolox.data.data_augment import ValTransform
from .yolox.data.datasets import COCO_CLASSES
from .yolox.exp import get_exp
from .yolox.utils import fuse_model, get_model_info, postprocess, setup_logger, vis

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


broker="172.27.34.65"
redis_server="172.27.34.65"
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


hi=HitbotInterface(92); #//92 is robotid
hi.net_port_initial()
ret=hi.initial(1,210); #
print(hi.is_connect())
print(hi.unlock_position())
hi.get_scara_param()
hi.wait_stop()
r.set("global_camera_xy",str(hi.x)+","+str(hi.y))
print(r.get("global_camera_xy"))




bounding_boxes_cords=None
class yolox_ros(yolox_py):
    def __init__(self) -> None:
        raw_image_topic = '/camera/camera/color/image_rect_raw'
        depth_image_topic = '/camera/camera/depth/image_rect_raw'
        depth_info_topic = '/camera/camera/depth/camera_info'


        # ROS2 init
        super().__init__('yolox_ros', load_params=False)

        #self.setting_yolox_exp()
        self.bridge = CvBridge()
        
        self.imshow_isshow=False
        self.sub_boxes = self.create_subscription(BoundingBoxesCords, "/yolox/bounding_boxes_cords", self.boxes_cords_callback, 1)
        
        self.intrinsics = None
        self.pix = None
        self.pix_grade = None

        #if (self.sensor_qos_mode):
        #    self.sub = self.create_subscription(Image,"/yolox/boxes_image",self.imageflow_callback, qos_profile_sensor_data)
        #else:
        self.sub = self.create_subscription(Image,"/yolox/boxes_image",self.imageflow_callback, 10)
    
    def imageflow_callback(self,msg:Image) -> None:
            global bounding_boxes
            img_rgb = self.bridge.imgmsg_to_cv2(msg,"bgr8")
            #outputs, img_info = self.predictor.inference(img_rgb)
            #logger.info("outputs: {},".format(outputs))
            #cv2.imwrite("/home/jimmy/Downloads/mushroomproject/ros2_web_server/static/mushroom.jpg",img_rgb)
            try:

                if (self.imshow_isshow):
                    cv2.imshow("YOLOX",img_rgb)
                    cv2.waitKey(1)
            except Exception as e:
                logger.error(e)
                pass
    def boxes_cords_callback(self, data):
        global bounding_boxes_cords
        bounding_boxes_cords=data.bounding_boxes
        logger.info(data.bounding_boxes)
        if r.get("start_scan")=="0":
            return;
        hi.get_scara_param()
        hi.wait_stop()
        #if 1:#r.get("mode")=="camera_ready":
        #    bounding_boxes_cords=data.bounding_boxes
            #r.set("mode","pickup_ready")

        r.set("mode","pickup_ready")
        
        if r.llen("queue")>0:
            ele=r.lpop("queue")
            #logger.info(ele)
            v=r.hget("detections",ele)
            xy=v.split(",")
            #logger.info(xy)
            if len(xy)>0:
                rett=hi.movel_xyz(int(xy[0]),int(xy[1]),hi.z,15,10)
                logger.info("rett:{}".format(rett))
                hi.wait_stop()
                r.hdel("detections",ele)
                hi.get_scara_param()
                hi.wait_stop()
                r.set("global_camera_xy",str(hi.x)+","+str(hi.y))
                r.set("mode","camera_ready")


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


