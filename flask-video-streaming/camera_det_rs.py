import io
from PIL import Image
import select
import cv2
#import v4l2capture
from base_camera import BaseCamera
from ultralytics import YOLO
from yolov8 import YOLOv8
import pyrealsense2 as rs


from rt_utils.rt_utils import preproc, vis
from rt_utils.rt_utils import BaseEngine
import numpy as np
import cv2
import time
import os

#model = YOLO("runs/slurm-619/train4/weights/best.pt")
#model_path = "weights/mushv8x-det.onnx"

#model_path = "weights/mushv8x-seg.transd.onnx"
#model_path = "weights/yolov8n-det.onnx"
#model_path = "weights/mushv8x-seg.trt"
model_path = "weights/best-seg.onnx"

yolov8_detector = YOLOv8(model_path, conf_thres=0.5, iou_thres=0.5)


pipeline = rs.pipeline()  # 定义流程pipeline
config = rs.config()  # 定义配置config
config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 30)
profile = pipeline.start(config)  # 流程开始
align_to = rs.stream.color  # 与color流对齐
align = rs.align(align_to)


def get_aligned_images():
    frames = pipeline.wait_for_frames()  # 等待获取图像帧
    aligned_frames = align.process(frames)  # 获取对齐帧
    aligned_depth_frame = aligned_frames.get_depth_frame()  # 获取对齐帧中的depth帧
    color_frame = aligned_frames.get_color_frame()  # 获取对齐帧中的color帧

    ############### 相机参数的获取 #######################
    intr = color_frame.profile.as_video_stream_profile().intrinsics  # 获取相机内参
    depth_intrin = aligned_depth_frame.profile.as_video_stream_profile(
    ).intrinsics  # 获取深度参数（像素坐标系转相机坐标系会用到）
    '''camera_parameters = {'fx': intr.fx, 'fy': intr.fy,
                         'ppx': intr.ppx, 'ppy': intr.ppy,
                         'height': intr.height, 'width': intr.width,
                         'depth_scale': profile.get_device().first_depth_sensor().get_depth_scale()
                         }'''

    # 保存内参到本地
    # with open('./intrinsics.json', 'w') as fp:
    # json.dump(camera_parameters, fp)
    #######################################################

    depth_image = np.asanyarray(aligned_depth_frame.get_data())  # 深度图（默认16位）
    depth_image_8bit = cv2.convertScaleAbs(depth_image, alpha=0.03)  # 深度图（8位）
    depth_image_3d = np.dstack(
        (depth_image_8bit, depth_image_8bit, depth_image_8bit))  # 3通道深度图
    color_image = np.asanyarray(color_frame.get_data())  # RGB图

    # 返回相机内参、深度参数、彩色图、深度图、齐帧中的depth帧
    return intr, depth_intrin, color_image, depth_image, aligned_depth_frame

class Camera(BaseCamera):
    """Requires python-v4l2capture module: https://github.com/gebart/python-v4l2capture"""

    video_source = "/dev/video0"

    @staticmethod
    def frames():
        #video = cv2.VideoCapture(Camera.video_source,cv2.CAP_V4L2)
        #size_x =640
        #size_y = 480
        #video.set(cv2.CAP_PROP_FRAME_WIDTH, size_x)
        #video.set(cv2.CAP_PROP_FRAME_HEIGHT, size_y)
        #video.set(cv2.CAP_PROP_FPS, 30)
        #bio = io.BytesIO()

        try:
            while True:
                intr, depth_intrin, color_image, depth_image, aligned_depth_frame = get_aligned_images()  # 获取对齐的图像与相机内参
                if not depth_image.any() or not color_image.any():
                    continue
                # Convert images to numpy arrays
                # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
                depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(
                    depth_image, alpha=0.03), cv2.COLORMAP_JET)
                # Stack both images horizontally
                images = np.hstack((color_image, depth_colormap))

                #1 result = model(img , agnostic_nms=True)[0]
                boxes, scores, class_ids = yolov8_detector(images)
                combined_img = yolov8_detector.draw_detections(images)
                #combined_img = pred.inference("./buf.jpg", conf=0.2, end2end=True)

                #print(combined_img)
                yield cv2.imencode('.jpg', combined_img)[1].tobytes()
        finally:
            #video.release()
            pipeline.stop()

