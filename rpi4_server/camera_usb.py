import io
from PIL import Image
import select
import cv2
#import v4l2capture
from base_camera import BaseCamera
from ultralytics import YOLO
#from yolov8 import YOLOv8


#from rt_utils.rt_utils import preproc, vis
#from rt_utils.rt_utils import BaseEngine
import numpy as np
import cv2
import time
import os
from loguru import logger
import redis
redis_server='172.27.34.62'
pool = redis.ConnectionPool(host=redis_server, port=6379, decode_responses=True,password='jimmy')
r = redis.Redis(connection_pool=pool)






#model = YOLO("/home/pi/yolomodel/yolo11n_ncnn_model")
model = YOLO("/home/pi/yolomodel/yolo11n_mushroom_ncnn_model")
model2 = YOLO("/home/pi/yolomodel/shape_yolo11_ncnn_model")
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
        size_y = 640
        video.set(cv2.CAP_PROP_FRAME_WIDTH, size_x)
        video.set(cv2.CAP_PROP_FRAME_HEIGHT, size_y)
        video.set(cv2.CAP_PROP_FPS, 30)
        bio = io.BytesIO()

        try:
            while video.isOpened():
                ret, img = video.read()
                frame=img

                if 1:# r.get("mode")=="adjust_camera_init" or r.get("mode")=="adjust_camera_done" :
                    detect_res=model(img,conf=0.8)
                    
                    frame=detect_res[0].plot()
                    boxes = detect_res[0].boxes.cpu().numpy()
                    xyxy = boxes.xyxy
                    classes = boxes.cls
                    confs = boxes.conf
                    #print(detect_res)
                    line_color=(255, 0, 255)
                    label_color=(255, 255, 255)
                    line_thickness=2
                    for (x1, y1, x2, y2), conf, class_id in zip(xyxy,confs,classes):
                        print(x1,y1,x2,y2)
                        left, top, right, bottom = int(x1), int(y1), int(x2), int(y2)
                        r.set("adjust_gripper_center",str(int((right + left-640) / 2))+","+str(int((top + bottom-640) / 2)))

                        width = right - left
                        height = bottom - top
                        center = (left + int((right - left) / 2), top + int((bottom - top) / 2))
                        label = "mushroom"
                        confidence = conf
                        label= label+",center:"+str(int((right + left) / 2))+","+str(int((top + bottom) / 2))
                            #if(label=="laptop"):
                        cv2.rectangle(img, (left, top), (right, bottom), (255, 0, 0), 2)
                        cv2.putText(img, label, center, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 1, cv2.LINE_AA)
                        break

                    #shape_res=model2(img)
                    #logger.info(shape_res)
                    #frame=shape_res[0].plot()

                #print(img)
                #cv2.imwrite("buf.jpg",img)
                if not ret:
                    break
                #1 result = model(img , agnostic_nms=True)[0]
                #boxes, scores, class_ids = yolov8_detector(img)
                #combined_img = yolov8_detector.draw_detections(img)
                #combined_img = pred.inference("./buf.jpg", conf=0.2, end2end=True)

                #print(combined_img)
                #frame=img
                yield cv2.imencode('.jpg', frame)[1].tobytes()
        finally:
            video.release()
