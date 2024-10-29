import io
import random
import threading

import redis
from PIL import Image
import select
import cv2


#import v4l2capture
from base_camera import BaseCamera
from ultralytics import YOLO
#from yolov8 import YOLOv8


#import xyz_publish
import cv2

import argparse
import os
import time
from loguru import logger
import torch
import numpy as np



ip=''
broker=''
redis_server='172.27.34.62'


pool = redis.ConnectionPool(host=redis_server, port=6379, decode_responses=True,password='jimmy')
r = redis.Redis(connection_pool=pool)




count=0
IMAGE_EXT = [".jpg", ".jpeg", ".webp", ".bmp", ".png"]

model = YOLO("/home/pi/yolomodel/yolo11n_ncnn_model")

#yolov8_detector = YOLOv8(model_path, conf_thres=0.5, iou_thres=0.5)
class Camera(BaseCamera):
    """Requires python-v4l2capture module: https://github.com/gebart/python-v4l2capture"""

    video_source = "/dev/video0"
    #def __init__(self):
    #    global ip
        #ip=newip
        #self.ip=ip

    @staticmethod
    def frames():

        #video = cv2.VideoCapture("http://192.168.0.100:5000/video_feed")
        video = cv2.VideoCapture("http://172.27.34.72:8000/stream.mjpg")
        #video = cv2.VideoCapture("http://"+ip+":5000/video_feed")
        #video = cv2.VideoCapture(Camera.video_source,cv2.CAP_V4L2)
        #video = v4l2capture.Video_device(Camera.video_source)

        size_x =640
        size_y = 480
        video.set(cv2.CAP_PROP_FRAME_WIDTH, size_x)
        video.set(cv2.CAP_PROP_FRAME_HEIGHT, size_y)
        video.set(cv2.CAP_PROP_FPS, 30)
        bio = io.BytesIO()



        try:
            while video.isOpened():
                ret, img = video.read()
                #print(img)
                #cv2.imwrite("buf.jpg",img)
                if not ret:
                    break
                global count
                count = (count + 1) % 10000
                #result=model(img)
                #frame=result[0].plot()
                #cv2.namedWindow("yolox", cv2.WINDOW_NORMAL)
                #cv2.imshow("yolox", result_frame)
                yield cv2.imencode('.jpg', img)[1].tostring()
        finally:
            video.release()
