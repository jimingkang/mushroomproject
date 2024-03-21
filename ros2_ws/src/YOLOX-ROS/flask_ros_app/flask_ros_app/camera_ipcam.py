import io
import random
import threading

import redis
from PIL import Image
import select
import cv2

#import move_subcribe
#import v4l2capture
from yolox_ros_py.base_camera import BaseCamera
#from ultralytics import YOLO
#from yolov8 import YOLOv8


#import xyz_publish
import cv2

import argparse
import os
import time
from loguru import logger
import torch
from yolox_ros_py.yolox.data.data_augment import ValTransform
from yolox_ros_py.yolox.data.datasets import COCO_CLASSES
from yolox_ros_py.yolox.exp import get_exp
from yolox_ros_py.yolox.utils import fuse_model, get_model_info, postprocess, vis





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

pool = redis.ConnectionPool(host=redis_server, port=6379, decode_responses=True,password='jimmy')
r = redis.Redis(connection_pool=pool)


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
        logger.info("in .camera_ipcam visual")
        ratio = img_info["ratio"]
        img = img_info["raw_img"]
        #if output is None:
        #    return img
        output = output.cpu()

        bboxes = output[:, 0:4]

        # preprocessing: resize
        bboxes /= ratio

        cls = output[:, 6]
        scores = output[:, 4] * output[:, 5]
        if 1:#r.get("mode")=="camera_ready":
            vis_res = vis(img, bboxes, scores, cls,count, cls_conf, self.cls_names)
            return vis_res,bboxes, scores, cls, self.cls_names
        else:
            return img




ip=''
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
        ip=broker.strip().replace("\n","")
        print(ip)
        #video = cv2.VideoCapture("http://192.168.0.100:5000/video_feed")
        video = cv2.VideoCapture("http://172.27.34.65:5000/video_feed")
        #video = cv2.VideoCapture("http://"+ip+":5000/video_feed")
        #video = cv2.VideoCapture(Camera.video_source,cv2.CAP_V4L2)
        #video = v4l2capture.Video_device(Camera.video_source)

        size_x =640
        size_y = 480
        video.set(cv2.CAP_PROP_FRAME_WIDTH, size_x)
        video.set(cv2.CAP_PROP_FRAME_HEIGHT, size_y)
        video.set(cv2.CAP_PROP_FPS, 30)
        bio = io.BytesIO()


        args = make_parser().parse_args()
        exp = get_exp(args.exp_file, args.name)
        predictor = main(exp, args)
        print(predictor)

        try:
            while video.isOpened():
                ret, img = video.read()
                #print(img)
                #cv2.imwrite("buf.jpg",img)
                if not ret:
                    break
                global count
                count = (count + 1) % 10000

                if ret:
                    if r.get("mode")=="camera_ready":
                        outputs, img_info = predictor.inference(img)
                        result_frame = predictor.visual(outputs[0], img_info, predictor.confthre)
                    else:
                        result_frame=img
                    #cv2.namedWindow("yolox", cv2.WINDOW_NORMAL)
                    #cv2.imshow("yolox", result_frame)
                yield cv2.imencode('.jpg', result_frame)[1].tobytes()
        finally:
            video.release()
