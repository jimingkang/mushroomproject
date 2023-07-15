import io
import random
import threading

from PIL import Image
import select
import cv2

import move_subcribe
#import v4l2capture
from base_camera import BaseCamera
#from ultralytics import YOLO
#from yolov8 import YOLOv8


#import xyz_publish
import cv2

import argparse
import os
import time
from loguru import logger
import torch
#from yolox.data.data_augment import ValTransform
#from yolox.data.datasets import COCO_CLASSES
#from yolox.exp import get_exp
#from yolox.utils import fuse_model, get_model_info, postprocess, vis

from paho.mqtt import client as mqtt_client


import pyrealsense2 as rs
import numpy as np
import cv2

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)
#config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)
#config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 30)
# Start streaming
# Start streaming
profile=pipeline.start(config)
#playback = pipeline_profile.get_device().as_playback()
#playback.set_real_time(False)
align_to = rs.stream.color  # 与color流对齐
align = rs.align(align_to)
count = 0
broker=''
try:
    for line in open("ip.txt"):
        if line[0:6] == "broker":
            broker = line[9:len(line)]
except:
    pass
print(broker)
#broker = '192.168.254.42'
#broker = '10.0.0.134'
port = 1883
topic = "/flask/scan"
topic4 = "/flask/downmove"
# generate client ID with pub prefix randomly
client_id = f'python-mqtt-{random.randint(0, 100)}'


IMAGE_EXT = [".jpg", ".jpeg", ".webp", ".bmp", ".png"]

def make_parser():
    parser = argparse.ArgumentParser("YOLOX Demo!")
    parser.add_argument(
        "--demo", default="webcam", help="demo type, eg. image, video and webcam"
    )
    parser.add_argument("-expn", "--experiment-name", type=str, default=None)
    parser.add_argument("-n", "--name", type=str, default=None, help="model name")

    parser.add_argument(
        "--path", default="./assets/dog.jpg", help="path to images or video"
    )
    parser.add_argument("--camid", type=int, default=0, help="webcam demo camera id")
    parser.add_argument(
        "--save_result",
        action="store_true",
        help="whether to save the inference result of image/video",
    )
    # exp file
    parser.add_argument(
        "-f",
        "--exp_file",
        default='./exps/example/yolox_voc/yolox_voc_s.py ',
        type=str,
        help="please input your experiment description file",
    )
    parser.add_argument("-c", "--ckpt", default=None, type=str, help="ckpt for eval")
    parser.add_argument(
        "--device",
        default="cpu",
        type=str,
        help="device to run our model, can either be cpu or gpu",
    )
    parser.add_argument("--conf", default=0.3, type=float, help="test conf")
    parser.add_argument("--nms", default=0.3, type=float, help="test nms threshold")
    parser.add_argument("--tsize", default=None, type=int, help="test img size")
    parser.add_argument(
        "--fp16",
        dest="fp16",
        default=False,
        action="store_true",
        help="Adopting mix precision evaluating.",
    )
    parser.add_argument(
        "--legacy",
        dest="legacy",
        default=False,
        action="store_true",
        help="To be compatible with older versions",
    )
    parser.add_argument(
        "--fuse",
        dest="fuse",
        default=False,
        action="store_true",
        help="Fuse conv and bn for testing.",
    )
    parser.add_argument(
        "--trt",
        dest="trt",
        default=True,
        action="store_true",
        help="Using TensorRT model for testing.",
    )
    return parser
def get_image_list(path):
    image_names = []
    for maindir, subdir, file_name_list in os.walk(path):
        for filename in file_name_list:
            apath = os.path.join(maindir, filename)
            ext = os.path.splitext(apath)[1]
            if ext in IMAGE_EXT:
                image_names.append(apath)
    return image_names

def main(exp, args):
    if not args.experiment_name:
        args.experiment_name = exp.exp_name

    file_name = os.path.join(exp.output_dir, args.experiment_name)
    os.makedirs(file_name, exist_ok=True)
    args.device = "gpu"
    logger.info("Args: {}".format(args))
    exp.test_conf = args.conf
    exp.nmsthre = args.nms
    if args.tsize is not None:
        exp.test_size = (args.tsize, args.tsize)

    model = exp.get_model()
    logger.info("Model Summary: {}".format(get_model_info(model, exp.test_size)))

    if args.device == "gpu":
        model.cuda()
        if args.fp16:
            model.half()  # to FP16
    model.eval()


    if args.trt:
        assert not args.fuse, "TensorRT model is not support model fusing!"
        trt_file = os.path.join(os.path.join("C:/Users/jkang7/Downloads/uh/project/MUSHROOM/YOLOX-main",file_name), "model_trt.pth")
        assert os.path.exists(
            trt_file
        ), "TensorRT model is not found!\n Run python3 tools/trt.py first!"
        model.head.decode_in_inference = False
        decoder = model.head.decode_outputs
        logger.info("Using TensorRT to inference")
    else:
        trt_file = None
        decoder = None

    predictor = Predictor(
        model, exp, COCO_CLASSES, trt_file, decoder,
        'gpu', False, False,
    )
    return predictor


class Predictor(object):
    def __init__(
        self,
        model,
        exp,
        #cls_names=COCO_CLASSES,
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
        print(str(height)+": width:"+str(width))
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
            logger.info("Infer time: {:.4f}s".format(time.time() - t0))
        return outputs, img_info

    def visual(self, output, img_info, cls_conf=0.35):
        ratio = img_info["ratio"]
        img = img_info["raw_img"]
        if output is None:
            return img
        output = output.cpu()

        bboxes = output[:, 0:4]

        # preprocessing: resize
        bboxes /= ratio

        cls = output[:, 6]
        scores = output[:, 4] * output[:, 5]
        global count
        count=count+1
        print(count,"count")

        vis_res = vis(img, bboxes, scores, cls,count, cls_conf, self.cls_names)
        return vis_res




def connect_mqtt():
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            i=1
            #print("xyx publish Connected to MQTT Broker!")
        else:
            i=0
            #print("Failed to connect, return code %d\n", rc)

    client = mqtt_client.Client(client_id)
    client.on_connect = on_connect
    client.connect(broker, port)
    return client

def get_405_aligned_images():

    frames = pipeline.try_wait_for_frames()
    while frames is None:
        frames= pipeline.try_wait_for_frames()
    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()
    depth_image = np.asanyarray(depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())
    if not depth_frame or not color_frame:
        return
    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)


    depth_colormap_dim = depth_colormap.shape
    color_colormap_dim = color_image.shape

    # If depth and color resolutions are different, resize color image to match depth image for display
    if depth_colormap_dim != color_colormap_dim:
        resized_color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]),
                                         interpolation=cv2.INTER_AREA)
        images = np.hstack((resized_color_image, depth_colormap))
    else:
        images = np.hstack((color_image, depth_colormap))

    # 返回相机内参、深度参数、彩色图、深度图、齐帧中的depth帧
    return color_image, depth_image, resized_color_image
    #return intr, depth_intrin, color_image, depth_image, resized_color_image
def get_aligned_images():

    frames = pipeline.wait_for_frames()  # 等待获取图像帧
    #playback.pause()
    aligned_frames = align.process(frames)  # 获取对齐帧
    aligned_depth_frame = aligned_frames.get_depth_frame()  # 获取对齐帧中的depth帧
    color_frame = aligned_frames.get_color_frame()  # 获取对齐帧中的color帧
    intr = color_frame.profile.as_video_stream_profile().intrinsics  # 获取相机内参
    depth_intrin = aligned_depth_frame.profile.as_video_stream_profile().intrinsics 
    depth_image = np.asanyarray(aligned_depth_frame.get_data())  # 深度图（默认16位）
    depth_image_8bit = cv2.convertScaleAbs(depth_image, alpha=0.03)  # 深度图（8位）
    depth_image_3d = np.dstack((depth_image_8bit, depth_image_8bit, depth_image_8bit))  # 3通道深度图
    color_image = np.asanyarray(color_frame.get_data())  # RGB图
    #playback.resume()
    return intr, depth_intrin, color_image, depth_image, aligned_depth_frame

#yolov8_detector = YOLOv8(model_path, conf_thres=0.5, iou_thres=0.5)
class Camera(BaseCamera):
    """Requires python-v4l2capture module: https://github.com/gebart/python-v4l2capture"""

    video_source = "/dev/video0"


    @staticmethod
    def frames():
        #video = cv2.VideoCapture(cv2.CAP_DSHOW)
        #video = cv2.VideoCapture("http://10.0.0.134:5000/video_feed")
        #video = cv2.VideoCapture("http://192.168.254.43:5000/video_feed")
        #video = cv2.VideoCapture(Camera.video_source,cv2.CAP_V4L2)
        #video = v4l2capture.Video_device(Camera.video_source)

        size_x =640
        size_y = 480
        ##video.set(cv2.CAP_PROP_FRAME_WIDTH, size_x)
        #video.set(cv2.CAP_PROP_FRAME_HEIGHT, size_y)
        #video.set(cv2.CAP_PROP_FPS, 30)
        bio = io.BytesIO()
        #client = connect_mqtt()
        #client.loop_start()
        #client.publish("/flask/serial", "0,0,0;")


        

        try:
            i=1

            while True:#video.isOpened():

                i = i + 1
                #ret, img = video.read()
                #color_image, depth_image, aligned_depth_frame = get_aligned_images()  # 获取对齐的图像与相机内参
                intr, depth_intrin, color_image, depth_image, aligned_depth_frame = get_aligned_images()  # 获取对齐的图像与相机内参
                if not depth_image.any() or not color_image.any():
                    continue
                #print(img)
                #cv2.imwrite("buf.jpg",img)
                #if not ret:
                #    break
                global count
                count = (count + 1) % 1000

        #        if 1:
        #            outputs, img_info = predictor.inference(color_image) #img
        #            result_frame = predictor.visual(outputs[0], img_info, predictor.confthre)
                    #cv2.namedWindow("yolox", cv2.WINDOW_NORMAL)
                    #if i<20:
                    #    cv2.imwrite("mushroom_ref.jpg", img_info)
                yield cv2.imencode('.jpg', color_image)[1].tobytes()
        finally:
            print("hello")
            #video.release()
