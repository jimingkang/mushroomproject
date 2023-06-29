import io
from PIL import Image
import select
import cv2
#import v4l2capture
from base_camera import BaseCamera
from ultralytics import YOLO
from yolov8 import YOLOv8
from yoloseg import YOLOSeg
from tracker import Tracker

from rt_utils.rt_utils import preproc, vis
from rt_utils.rt_utils import BaseEngine
import numpy as np
import cv2
import time
import os



#model_path = "weights/yolov8n-det.onnx"
model_path = "weights/yolov8x-seg.onnx"




yoloseg= YOLOSeg(model_path, conf_thres=0.5, iou_thres=0.5)
class Camera(BaseCamera):
    """Requires python-v4l2capture module: https://github.com/gebart/python-v4l2capture"""

    video_source = "/dev/video0"

    @staticmethod
    def frames():
        video = cv2.VideoCapture("http://10.0.0.251:5000/video_feed")
        #video = cv2.VideoCapture(Camera.video_source,cv2.CAP_V4L2)
        #video = v4l2capture.Video_device(Camera.video_source)
        # Suggest an image size. The device may choose and return another if unsupported
        size_x =640
        size_y = 480
        video.set(cv2.CAP_PROP_FRAME_WIDTH, size_x)
        video.set(cv2.CAP_PROP_FRAME_HEIGHT, size_y)
        video.set(cv2.CAP_PROP_FPS, 30)
        bio = io.BytesIO()

        try:
            while video.isOpened():
                ret, frame = video.read()
                if not ret:
                    break
                boxes, scores, class_ids, masks = yoloseg(frame)
                combined_img = yoloseg.draw_masks(frame)
                #print(combined_img)
                yield cv2.imencode('.jpg', combined_img)[1].tobytes()
        finally:
            video.release()
