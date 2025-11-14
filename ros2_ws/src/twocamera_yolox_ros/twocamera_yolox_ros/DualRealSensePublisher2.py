#!/usr/bin/env python3
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
#import RPi.GPIO as GPIO

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
from .camera_ipcam import Predictor

import pyrealsense2 as rs
from numpy import empty
import torch
import torch.backends.cudnn as cudnn
from yolox.data.data_augment import ValTransform
from yolox.data.datasets import COCO_CLASSES
from yolox.exp import get_exp
from yolox.utils import fuse_model, get_model_info, postprocess, setup_logger, vis

from roboflow import Roboflow
import supervision as sv
import cv2


from ultralytics import YOLO
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
#from yolox_ros_py_utils.utils import yolox_py
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String,Int32
from scipy.spatial.transform import Rotation as R
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
broker="172.23.66.117"
redis_server='172.23.248.33'

pool = redis.ConnectionPool(host=redis_server, port=6379, decode_responses=True, password='jimmy')
r = redis.Redis(connection_pool=pool)

class DualRealSensePublisher2(Node):
    def __init__(self):
        super().__init__('dual_realsense_publisher')
        self.infer_model = YOLO("/home/jimmy/Downloads/train15_yolo11.engine")
        self.bridge = CvBridge()
        self.d435_intrinsics = None
        self.d405_intrinsics = None
        serials = ['027422070780','128422272136']
        #serials = ['128422272136']# 128422272400
        self.pipelines = []
        self.aligns = []   # 每个相机独立一个 align 对象

        # 初始化 pipeline 和 align
        for s in serials:
            pipeline = rs.pipeline()
            cfg = rs.config()
            cfg.enable_device(s)
            cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)
            cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)
            pipeline.start(cfg)
            profile = pipeline.get_active_profile()
            # Get color stream intrinsics
            color_stream = profile.get_stream(rs.stream.color)  # rs.video_stream_profile
            cameraInfo = color_stream.as_video_stream_profile().get_intrinsics()
            if s == '027422070780':

                try:
                    if self.d435_intrinsics:
                        return
                    self.d435_intrinsics = rs.intrinsics()
                    self.d435_intrinsics.width = cameraInfo.width
                    self.d435_intrinsics.height = cameraInfo.height
                    self.d435_intrinsics.ppx = cameraInfo.ppx
                    self.d435_intrinsics.ppy = cameraInfo.ppy
                    self.d435_intrinsics.fx = cameraInfo.fx
                    self.d435_intrinsics.fy = cameraInfo.fy
                    self.d435_intrinsics.model = cameraInfo.model
                    self.d435_intrinsics.coeffs = cameraInfo.coeffs
                except CvBridgeError as e:
                    print(e)
                    return
            else:
                try:
                    if self.d405_intrinsics:
                        return
                    self.d405_intrinsics = rs.intrinsics()
                    self.d405_intrinsics.width = cameraInfo.width
                    self.d405_intrinsics.height = cameraInfo.height
                    self.d405_intrinsics.ppx = cameraInfo.ppx
                    self.d405_intrinsics.ppy = cameraInfo.ppy
                    self.d405_intrinsics.fx = cameraInfo.fx
                    self.d405_intrinsics.fy = cameraInfo.fy
                    self.d405_intrinsics.model = cameraInfo.model
                    self.d405_intrinsics.coeffs = cameraInfo.coeffs
                except CvBridgeError as e:
                    print(e)
                    return

                
            # align depth 到 color
            align_to = rs.stream.color
            align = rs.align(align_to)
            self.pipelines.append(pipeline)
            self.aligns.append(align)

        self.pub1 = self.create_publisher(Image, '/d435/color/image_raw', 10)
        self.pub2 = self.create_publisher(Image, '/d405/color/image_raw', 10)

        self.d435_pub_bounding_boxes = self.create_publisher(String,"/d435/yolox/bounding_boxes", 1)
        self.d435_pub_boxes_img = self.create_publisher(Image,"/d435/yolox/boxes_image", 10)
        #self.d435_sub_info = self.create_subscription(CameraInfo, self.d435_depth_info_topic, self.D435_imageDepthInfoCallback, qos)

        self.d405_adj_pub_bounding_boxes = self.create_publisher(String,"/d405/yolox/adj_bounding_boxes", 1)
        self.d405_pub_boxes_img = self.create_publisher(Image,"/d405/yolox/boxes_image", 10)
        #self.d405_sub_info = self.create_subscription(CameraInfo, self.d405_depth_info_topic, self.d405_imageDepthInfoCallback, qos)



        self.setting_yolox_exp()



        self.timer = self.create_timer(0.03, self.timer_callback)


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
            model.load_state_dict(ckpt["model"],strict=False)
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
    def timer_callback(self):
        for j, (pipe, align) in enumerate(zip(self.pipelines, self.aligns)):
            
            print(j)
            frames = pipe.wait_for_frames()
            aligned_frames = align.process(frames)

            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue

            color_img = np.asanyarray(color_frame.get_data())
            depth_img = np.asanyarray(depth_frame.get_data())

            color_msg = self.bridge.cv2_to_imgmsg(color_img, encoding='bgr8')
            depth_msg = self.bridge.cv2_to_imgmsg(depth_img, encoding='16UC1')
            (self.pub1 if j == 0 else self.pub2).publish(color_msg)

            #outputs, img_info = self.predictor.inference(color_img) #yolox
            detect_res = self.infer_model.predict(color_img, conf=0.50, verbose=False) #yolo11
            result = detect_res[0]

            logger.info("mode={},mode==adjust_ready,{}".format(r.get("mode"),r.get("mode")=="adjust_ready"))
            bbox=String()
            #if  (outputs is not None) : #and r.get("mode")=="camera_ready":# and r.get("scan")=="start" :#
            if result is None :
                logger.info("No detections returned from predictor.")
                continue  # or continue
            else:
                boxes = result.boxes  # Boxes object
                if boxes is not None and len(boxes) > 0:
                    for box in boxes:
                        # box.xyxy 是 [x1, y1, x2, y2]
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        conf = float(box.conf[0])     # 置信度
                        cls_id = int(box.cls[0])      # 类别ID

                        # 打印结果
                        print(f"Detected class={cls_id}, conf={conf:.2f}, box=({x1},{y1},{x2},{y2})")

                        # 在图像上画框
                        cv2.rectangle(color_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        cv2.putText(color_img, f"cls={cls_id} conf={conf:.2f}", (x1, y1 - 10),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
            if j==0 :
                print(f"{j==0} d435 image publish")
                self.d435_pub_boxes_img.publish(self.bridge.cv2_to_imgmsg(color_img, encoding='bgr8'))
                if (r.get("mode")=="camera_ready"):
                    if bbox.data=="":
                        logger.info("no valid bbox from d435")
                        continue
                    self.d435_pub_bounding_boxes.publish(bbox)
                    logger.info(" mushroom xyz in side camera: {}".format(bbox.data))

            elif j==1 :
                print(f"{j==0} d405 image publish")
                self.d405_pub_boxes_img.publish(self.bridge.cv2_to_imgmsg(color_img, encoding='bgr8'))
                if  (r.get("mode")=="adjust_ready"):
                    if bbox.data=="" :
                        logger.info("no valid bbox from d405")
                        continue
                    logger.info("mushroom xyz in top camera before rotation:{},".format(bbox.data))
                    X,Y,Z=[float(x) for x in bbox.data.split(",")]
                    self.d405_adj_pub_bounding_boxes.publish(bbox)
            bbox.data=""


def main():
    rclpy.init()
    node = DualRealSensePublisher2()
    rclpy.spin(node)
    node.stop_pipelines()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
