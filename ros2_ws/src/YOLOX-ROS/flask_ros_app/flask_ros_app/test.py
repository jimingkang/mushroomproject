import sys
import numpy as np
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
from std_msgs.msg import String
from bboxes_ex_msgs.msg import BoundingBoxes,BoundingBox

from sensor_msgs.msg import CameraInfo

from .camera_ipcam import Predictor,Camera
from importlib import import_module
import os
from flask import Flask, render_template, Response, jsonify
from threading import Event
import requests
from flask_mqtt import Mqtt

import redis
import serial
import datetime
import time
import os
from time import sleep
import time
from flask_cors import CORS, cross_origin

import signal
from rclpy.node import Node
from std_msgs.msg import String
import threading
from flask import Flask
from sensor_msgs.msg import Image
from cv_bridge import CvBridge,CvBridgeError # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import pyrealsense2 as rs2
if (not hasattr(rs2, 'intrinsics')):
    import pyrealsense2.pyrealsense2 as rs2


import io
import random
import threading

import redis
from PIL import Image
import select
import cv2


#import v4l2capture
from .base_camera import BaseCamera
#from ultralytics import YOLO
#from yolov8 import YOLOv8


import cv2

import argparse
import os
import time
from loguru import logger
import torch
from yolox.data.data_augment import ValTransform
from yolox.data.datasets import COCO_CLASSES
from yolox.exp import get_exp
from yolox.utils import fuse_model, get_model_info, postprocess, vis

from paho.mqtt import client as mqtt_client

ip=''
broker=''
redis_server=''
try:
    for line in open("../ip.txt"):
        if line[0:6] == "broker":
            broker = line[9:len(line)-1]
        if line[0:6] == "reddis":
            redis_server=line[9:len(line)-1]
except:
    pass
print(broker+" "+redis_server)
print(broker)



redis_server="172.27.34.65"
pool = redis.ConnectionPool(host=redis_server, port=6379, decode_responses=True,password='jimmy')
r = redis.Redis(connection_pool=pool)
app = Flask(__name__)
app.config['MQTT_BROKER_URL'] = "172.27.34.65"
app.config['MQTT_BROKER_PORT'] = 1883
app.config['MQTT_USERNAME'] = ''  # Set this item when you need to verify username and password
app.config['MQTT_PASSWORD'] = ''  # Set this item when you need to verify username and password
app.config['MQTT_KEEPALIVE'] = 5  # Set KeepAlive time in seconds
app.config['MQTT_TLS_ENABLED'] = False  # If your server supports TLS, set it True
cors = CORS(app)
app.config['CORS_HEADERS'] = 'Content-Type'
mqtt_client = Mqtt(app)
#prev_sigint_handler = signal.signal(signal.SIGINT, sigint_handler)
port = 1883
topic = '/flask/mqtt'
topic2 = '/flask/xyz'
topic3 = '/flask/serial'
topic4 = "/flask/downmove"
# generate client ID with pub prefix randomly
client_id = f'python-mqtt-{random.randint(0, 100)}'

count=0
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


def getPredictor(exp, args):
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
        trt_file = os.path.join(file_name, "model_trt.pth")
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




bounding_boxes=None
class TestPublisher(Node):
    def __init__(self):
        super().__init__('test_publisher')
        self.bridge = CvBridge()
        #self.publisher = self.create_publisher(String, 'flask_pub_topic', 10)
        #self.subscription = self.create_subscription(BoundingBoxes, '/yolox/bounding_boxes', self.listener2_callback, 10)
        self.subscription = self.create_subscription(Image, '/yolox/image_raw', self.listener_callback, 10)

        #self.subscription # prevent unused variable warning
    def listener_callback(self, data):
      """
      Callback function.
      """
      # Display the message on the console
      #self.get_logger().info(data.)
      # Convert ROS Image message to OpenCV image
      current_frame = self.bridge.imgmsg_to_cv2(data)
      # Display image
      cv2.imshow("camera", current_frame)
      cv2.waitKey(1)

    def listener2_callback(self, data):
        global bounding_boxes
        #bounding_boxe=None
        if r.get("mode")=="camera_ready":
            bounding_boxes=data.bounding_boxes
            r.set("mode","pickup_ready")
        #print(data)
        # Display the message on the console
        #for box in bounding_boxes:
        #    print("probability,%4.2f,%s,x=%4.2f,y=%4.2f",box.probability,box.class_id,(box.xmin+box.xmax)/2,(box.ymin+box.ymax)/2)





class ImageSubscriber(Node):
    def imageRawCallback(self, data):
        global bounding_boxes
        cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
        if r.get("mode")=="camera_ready":
            outputs, img_info = predictor.inference(cv_image)
            result_frame = predictor.visual(outputs[0], img_info, predictor.confthre)
        else:
            result_frame=cv_image
        cv2.imshow("camera", result_frame)
        cv2.waitKey(1)

    
    def imageDepthCallback(self, data):
        global bounding_boxes
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
            # pick one pixel among all the pixels with the closest range:
            indices = np.array(np.where(cv_image == cv_image[cv_image > 0].min()))[:,0]
            print(bounding_boxes)
            if bounding_boxes is not None:
                for box in bounding_boxes:
                    print("probability,%4.2f,%s,x=%4.2f,y=%4.2f",box.probability,box.class_id,(box.xmin+box.xmax)/2,(box.ymin+box.ymax)/2)

                    pix = (indices[1], indices[0])
                    pix = (int((box.xmin+box.xmax)/2),int((box.ymin+box.ymax)/2))
                    self.pix = pix
                    line = '\rDepth at pixel(%3d, %3d): %7.1f(mm).' % (pix[0], pix[1], cv_image[pix[1], pix[0]])
                    if self.intrinsics:
                        depth = cv_image[pix[1], pix[0]]
                        result = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [pix[0], pix[1]], depth)
                        line += '  Coordinate: %8.2f %8.2f %8.2f.' % (result[0], result[1], result[2])
                    if (not self.pix_grade is None):
                        line += ' Grade: %2d' % self.pix_grade
                    line += '\r'
                    print(line)
            r.set("mode","pickup_ready")
            bounding_boxe=None
        except CvBridgeError as e:
            print(e)
            return
        except ValueError as e:
            return
    
    def imageDepthInfoCallback(self, cameraInfo):
        try:
            if self.intrinsics:
                return
            self.intrinsics = rs2.intrinsics()
            self.intrinsics.width = cameraInfo.width
            self.intrinsics.height = cameraInfo.height
            self.intrinsics.ppx = cameraInfo.k[2]
            self.intrinsics.ppy = cameraInfo.k[5]
            self.intrinsics.fx = cameraInfo.k[0]
            self.intrinsics.fy = cameraInfo.k[4]
            if cameraInfo.distortion_model == 'plumb_bob':
                self.intrinsics.model = rs2.distortion.brown_conrady
            elif cameraInfo.distortion_model == 'equidistant':
                self.intrinsics.model = rs2.distortion.kannala_brandt4
            self.intrinsics.coeffs = [i for i in cameraInfo.d]
        except CvBridgeError as e:
            print(e)
            return
        
    def __init__(self,depth_image_topic, depth_info_topic,raw_image_topic):
        super().__init__('image_subscriber')
        self.bridge = CvBridge()
        self.sub_rawImage = self.create_subscription(Image, raw_image_topic, self.imageRawCallback, 1)
        self.sub = self.create_subscription(Image, depth_image_topic, self.imageDepthCallback, 1)
        self.sub_info = self.create_subscription(CameraInfo, depth_info_topic, self.imageDepthInfoCallback, 1)
        self.intrinsics = None
        self.pix = None
        self.pix_grade = None
        args = make_parser().parse_args()
        exp = get_exp(args.exp_file, args.name)
        predictor = getPredictor(exp, args)

    


def main(args=None):
  raw_image_topic = '/camera/camera/color/image_rect_raw'
  depth_image_topic = '/camera/camera/depth/image_rect_raw'
  depth_info_topic = '/camera/camera/depth/camera_info'
  
  
  # Initialize the rclpy library
  rclpy.init(args=args)

  test_Publisher = TestPublisher()
  image_subscriber = ImageSubscriber(depth_image_topic, depth_info_topic,raw_image_topic)

  executor = rclpy.executors.MultiThreadedExecutor()
  executor.add_node(test_Publisher)
  executor.add_node(image_subscriber)
  executor_thread=threading.Thread(target=executor.spin, daemon=True)

  executor_thread.start()

  # Create the node
  # Spin the node so the callback function is called.
  #rclpy.spin(test_Publisher)
  #test_Publisher.destroy_node()
  #rclpy.spin(image_subscriber)
  #image_subscriber.destroy_node()
  executor_thread.join()
  rclpy.shutdown()

  
if __name__ == '__main__':
  main()
