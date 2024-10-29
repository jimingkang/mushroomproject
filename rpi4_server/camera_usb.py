import io
from PIL import Image
import select
import cv2
#import v4l2capture
from base_camera import BaseCamera
#from ultralytics import YOLO
#from yolov8 import YOLOv8


#from rt_utils.rt_utils import preproc, vis
#from rt_utils.rt_utils import BaseEngine
import numpy as np
import cv2
import time
import os



#model = YOLO("runs/slurm-619/train4/weights/best.pt")
#model_path = "weights/mushv8x-det.onnx"

model_path = "weights/mush-yolov8-x.onnx"
#model_path = "weights/mushv8x-seg.transd.onnx"
#model_path = "weights/yolov8n-det.onnx"
#model_path = "weights/mushv8x-seg.trt"




#yolov8_detector = YOLOv8(model_path, conf_thres=0.5, iou_thres=0.5)
class Camera(BaseCamera):
    """Requires python-v4l2capture module: https://github.com/gebart/python-v4l2capture"""

    video_source = "/dev/video0"

    @staticmethod
    def frames():
        video = cv2.VideoCapture(Camera.video_source,cv2.CAP_V4L2)
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
                ret, img = video.read()
                #print(img)
                #cv2.imwrite("buf.jpg",img)
                if not ret:
                    break
                #1 result = model(img , agnostic_nms=True)[0]
                #boxes, scores, class_ids = yolov8_detector(img)
                #combined_img = yolov8_detector.draw_detections(img)
                #combined_img = pred.inference("./buf.jpg", conf=0.2, end2end=True)

                #print(combined_img)
                yield cv2.imencode('.jpg', img)[1].tobytes()
        finally:
            video.release()
