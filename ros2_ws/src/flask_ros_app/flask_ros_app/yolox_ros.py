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

import time
import os
from time import sleep
import math
from flask_cors import CORS, cross_origin


import time
from .HitbotInterface import HitbotInterface
import redis
from paho.mqtt import client as mqtt_client

import numpy as np
import cv2
from loguru import logger



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

from sensor_msgs.msg import  JointState




i = 0




broker="172.27.34.62"
redis_server="172.27.34.62"
print(broker)
print(redis_server)
pool = redis.ConnectionPool(host=redis_server, port=6379, decode_responses=True, password='jimmy')
r = redis.Redis(connection_pool=pool)

app = Flask(__name__)
app.config['DEBUG'] = False

cors = CORS(app)
app.config['CORS_HEADERS'] = 'Content-Type'


y = 0




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
        self.img_rgb=np.zeros( (512,512,3), dtype=np.uint8);#cv2.imread('/home/jimmy/Downloads/mushroomproject/ros2_ws/src/flask_ros_app/flask_ros_app/mushroom.jpg', 0)
        self.pub_boxes_img = self.create_publisher(Image,"/yolox/boxes_image", 10)
        self.sub_boxes = self.create_subscription(BoundingBoxesCords, "/yolox/bounding_boxes_cords", self.boxes_cords_callback, 1)
        self.hitbot_end_xyz_pub = self.create_publisher(String, "/hitbot_end_xyz", 10)
        #self.rpi_sub_boxes = self.create_subscription(String, "/yolox/move/adjust/xy", self.boxes_move_adjust_callback, 1)
        #self.motor_goal = self.create_subscription(MotorGoal, "/motor_goal", self.motor_goal_callback, 1)
        
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 1)
        #self.joint_state_pub = self.create_publisher(MotorState, '/motor_state', 1)
        self.gripper_detected_publisher = self.create_publisher(String, '/yolox/move/detected', 1)
        self.gripper_publisher = self.create_publisher(String, '/yolox/gripper_hold', 1)
        self.gripper_open_pub = self.create_publisher(String, '/yolox/gripper_open', 1)
        
        self.intrinsics = None
        self.pix = None
        self.pix_grade = None
        self.scan_i=0;
        self.scan_j=0;
        self.count=0;
        self.pre_count=-1;

        self.hitbot_x  = "0"
        self.hitbot_y  = "0"
        self.hitbot_z  = "0"
        self.hitbot_r = "0"

        #if (self.sensor_qos_mode):
        #    self.sub = self.create_subscription(Image,"/yolox/boxes_image",self.imageflow_callback, qos_profile_sensor_data)
        #else:
        self.sub = self.create_subscription(Image,"/camera/camera/color/image_rect_raw",self.imageflow_callback, 10) #yolox/boxes_image


        self.subscription = self.create_subscription(
            String,
            '/hitbot_x',
            self.hitbot_x_callback,
            10
        )

        self.subscription = self.create_subscription(
            String,
            '/hitbot_y',
            self.hitbot_y_callback,
            10
        )

        self.subscription = self.create_subscription(
            String,
            '/hitbot_z',
            self.hitbot_z_callback,
            10
        )

        self.subscription = self.create_subscription(
            String,
            '/hitbot_r',
            self.hitbot_r_callback,
            10
        )



    def hitbot_x_callback(self, msg):
        self.hitbot_x  = msg.data

    def hitbot_y_callback(self, msg):
        self.hitbot_y = msg.data

    def hitbot_z_callback(self, msg):
        self.hitbot_z  = msg.data

    def hitbot_r_callback(self, msg):
        self.hitbot_r = msg.data

    
    def imageflow_callback(self,msg:Image) -> None:
            global bounding_boxes
            self.img_rgb =self.bridge.imgmsg_to_cv2(msg,"bgr8")

            rett=0

            self.pre_count=self.pre_count+1;
            #if self.count==self.pre_count:
            #    r.set("scan","start");

            if r.get("scan")=="start":

                #self.scan_i=self.scan_i+1;
                self.pre_count=self.count;
                self.count=self.count+1;
    



                scan_msg = String()
                scan_msg.data="600,0,0,-63"
                self.hitbot_end_xyz_pub.publish(scan_msg)
                time.sleep(2)
                r.set("mode","camera_ready");
                time.sleep(1)
                r.set("mode","camera_stop");

                scan_msg.data="500,0,0,-63"
                self.hitbot_end_xyz_pub.publish(scan_msg)
                time.sleep(2)
                r.set("mode","camera_ready");
                time.sleep(1)
                r.set("mode","camera_stop");
                
                scan_msg.data="400,0,0,-63"
                self.hitbot_end_xyz_pub.publish(scan_msg)
                time.sleep(2)
                r.set("mode","camera_ready");
                time.sleep(1)
                r.set("mode","camera_stop");

                scan_msg.data="300,0,0,-63"
                self.hitbot_end_xyz_pub.publish(scan_msg)
                time.sleep(2)
                r.set("mode","camera_ready");
                time.sleep(1)
                r.set("mode","camera_stop");

                scan_msg = String()
                scan_msg.data="300,100,0,-63"
                self.hitbot_end_xyz_pub.publish(scan_msg)
                time.sleep(2)
                r.set("mode","camera_ready");
                time.sleep(1)
                r.set("mode","camera_stop");

                scan_msg.data="300,200,0,-63"
                self.hitbot_end_xyz_pub.publish(scan_msg)
                time.sleep(2)
                r.set("mode","camera_ready");
                time.sleep(1)
                r.set("mode","camera_stop");
                
                scan_msg.data="300,300,0,-63"
                self.hitbot_end_xyz_pub.publish(scan_msg)
                time.sleep(2)
                r.set("mode","camera_ready");
                time.sleep(1)
                r.set("mode","camera_stop");

                scan_msg.data="300,400,0,-63"
                self.hitbot_end_xyz_pub.publish(scan_msg)
                time.sleep(2)
                r.set("mode","camera_ready");
                time.sleep(1)
                r.set("mode","camera_stop");

                r.set("scan","stop");


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

        logger.info(r.llen("queue"))  
        logger.info("stop,{}".format(r.get("scan")=="stop")) 
        hlen=r.hlen("detections") 
        if r.get("scan")=="stop" and hlen>0 : #r.get("scan")=="stop" and
            #ele=r.lpop("queue") 
            #v=r.hget("detections",ele)
            print(r.hgetall("detections"))
            for key in r.hkeys("detections"):
                v=r.hget("detections",key)
                xy=[]
                if v is not None:
                    xy=v.split(",")
                    logger.info(xy)
                if v is not None and len(xy)>0:
                    scan_msg = String()
                    scan_msg.data=str(xy[0])+","+str(xy[1])+","+str(int(self.hitbot_z) -20)+",-63"
                    self.hitbot_end_xyz_pub.publish(scan_msg)  
                    logger.info(f"movedown current location :{xy[0]},{xy[1]},")
                    r.hdel("detections",key)
                    r.set("mode","adjust_camera_init")
                    adjust_gripper_center=r.get("adjust_gripper_center")
                    
                    while 0:#adjust_gripper_center!="":
                        adjust=adjust_gripper_center.split(",")
                        x=int(adjust[0])
                        y=int(adjust[1])

                        r.set("mode","adjust_camera_ready")
                        logger.info("adjust_camera_ready (pixel) : {}, {},".format(x,y))
                        if abs(x)>20 or abs(y)>20: #r.get("scan")=="stop" and

                                scan_msg = String()
                                scan_msg.data=str(self.hitbot_x +x/10)+","+self.hitbot_y +","+str(self.hitbot_z -10)+",-63"
                                self.hitbot_end_xyz_pub.publish(scan_msg)  
                                
                                if rett>0:
                                    break
                        r.set("mode","adjust_camera_done")
                        time.sleep(1)
                    time.sleep(1)
                    gripper_msg = String()
                    gripper_msg.data = '%d,%d,%d' %(int(xy[0]),int(xy[1]),0) 
                    self.gripper_publisher.publish(gripper_msg)
                    time.sleep(1)
                    logger.info(r.get("mode"))
                    scan_msg = String()
                    scan_msg.data=self.hitbot_x +","+self.hitbot_y +","+str(0)+",-63"
                    self.hitbot_end_xyz_pub.publish(scan_msg) 
                    time.sleep(1)

                    gripper_msg2 = String()
                    gripper_msg2.data = 'gripper open' 
                    self.gripper_open_pub.publish(gripper_msg2)
                    time.sleep(2)


        r.set("mode","camera_ready")
        r.set("scan","start")
        img_rgb_pub = self.bridge.cv2_to_imgmsg(self.img_rgb,"bgr8")
        self.pub_boxes_img.publish(img_rgb_pub)
    def motor_goal_callback(self, msg):
        data=msg.data
        logger.info(data)

    def boxes_move_adjust_callback(self, msg):
        data=msg.data
        logger.info(data)
        adjust=data.split(",")
        x=int(adjust[0])
        y=int(adjust[1])

        r.set("mode","adjust_camera_ready")
        logger.info("adjust_camera_ready (pixel) : {}, {},".format(x,y))
        if abs(x)>20 or abs(y)>20: #r.get("scan")=="stop" and
                #rett=hi.movel_xyz(hi.x+int(x),hi.y+int(y),hi.z-10,-63,80)
                #hi.wait_stop()
                #back_z=hi.z
                logger.info("movedown current location :{},{},{},".format(0,0,0))
        r.set("mode","adjust_camera_done")
        r.set("adjust_gripper_center",str(int((640) / 2))+","+str(int((640) / 2)))

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


