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

#model = YOLO("/home/pi/yolomodel/yolo11n_ncnn_model")

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
                yield cv2.imencode('.jpg', img)[1].tobytes()
        finally:
            video.release()

class Predictor(object):
    def __init__(
        self,
        model,
        exp,
        cls_names=COCO_CLASSES,
        trt_file=None,
        decoder=None,
        device="cpu",
        fp16=False,
        legacy=False,
    ):
        self.model = model
        self.cls_names = cls_names
        self.decoder = decoder
        self.num_classes = exp.num_classes
        self.confthre = exp.test_conf
        self.nmsthre = exp.nmsthre
        self.test_size = exp.test_size
        self.device = device
        self.fp16 = fp16
        self.preproc = ValTransform(legacy=legacy)
        if trt_file is not None:
            from torch2trt import TRTModule

            model_trt = TRTModule()
            model_trt.load_state_dict(torch.load(trt_file))

            x = torch.ones(1, 3, exp.test_size[0], exp.test_size[1]).cuda()
            self.model(x)
            self.model = model_trt

    def inference(self, img):
        img_info = {"id": 0}
        if isinstance(img, str):
            img_info["file_name"] = os.path.basename(img)
            img = cv2.imread(img)
        else:
            img_info["file_name"] = None

        height, width = img.shape[:2]
        img_info["height"] = height
        img_info["width"] = width
        img_info["raw_img"] = img

        ratio = min(self.test_size[0] / img.shape[0], self.test_size[1] / img.shape[1])
        img_info["ratio"] = ratio

        img, _ = self.preproc(img, None, self.test_size)
        img = torch.from_numpy(img).unsqueeze(0)
        img = img.float()
        if self.device == "gpu":
            img = img.cuda()
            if self.fp16:
                img = img.half()  # to FP16

        with torch.no_grad():
            t0 = time.time()
            outputs = self.model(img)
            if self.decoder is not None:
                outputs = self.decoder(outputs, dtype=outputs.type())
            outputs = postprocess(
                outputs, self.num_classes, self.confthre,
                self.nmsthre, class_agnostic=True
            )
            logger.info("in .camera Infer time: {:.4f}s".format(time.time() - t0))
        return outputs, img_info

    def visual(self, output, img_info, cls_conf=0.35):
        #logger.info("new test in .camera_ipcam visual")
        ratio = img_info["ratio"]
        img = img_info["raw_img"]
        if output is None:
            return img,None,None,None,None,None
        output = output.cpu()

        bboxes = output[:, 0:4]

        # preprocessing: resize
        bboxes /= ratio

        cls = output[:, 6]
        scores = output[:, 4] * output[:, 5]
        if 1:#r.get("mode")=="camera_ready":
            vis_res, track_ids = vis(img, bboxes, scores, cls,count, cls_conf, self.cls_names)

            return vis_res,bboxes, scores, cls, self.cls_names,track_ids
        else:
            return img,None,None,None,None,None
