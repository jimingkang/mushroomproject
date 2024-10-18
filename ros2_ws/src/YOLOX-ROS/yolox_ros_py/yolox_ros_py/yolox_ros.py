#!/usr/bin/env python
import argparse
from importlib import import_module
import os

import redis
from flask import Flask, render_template, Response, jsonify

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
import RPi.GPIO as GPIO

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
from yolox_ros_py.camera_ipcam import Predictor

import pyrealsense2 as pyrs
from numpy import empty
import torch
import torch.backends.cudnn as cudnn
from .yolox.data.data_augment import ValTransform
from .yolox.data.datasets import COCO_CLASSES
from .yolox.exp import get_exp
from .yolox.utils import fuse_model, get_model_info, postprocess, setup_logger, vis

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from cv_bridge import CvBridge,CvBridgeError
from sensor_msgs.msg import Image,PointCloud2,PointField
from rclpy.qos import qos_profile_sensor_data
from bboxes_ex_msgs.msg import BoundingBoxes,BoundingBoxesCords
from bboxes_ex_msgs.msg import BoundingBox,BoundingBoxCord
from .yolox_ros_py_utils.utils import yolox_py
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String
#import orbslam3
from slam import Mapp,Frame
from scipy.spatial.transform import Rotation as R
# camera intrinsics
W, H = 848-12, 480+10
F = 430.79
K = np.array([[F,0,W//2],[0,F,H//2],[0,0,1]])
mapp = Mapp(W, H)
frame=None



# import camera driver
if os.environ.get('CAMERA'):
    Camera = import_module('camera_' + os.environ['CAMERA']).Camera
else:
    from .camera_ipcam import Camera

import pyrealsense2 as rs2
if (not hasattr(rs2, 'intrinsics')):
    import pyrealsense2.pyrealsense2 as rs2

i = 0



broker="172.27.34.62"
redis_server="172.27.34.62"


pool = redis.ConnectionPool(host=redis_server, port=6379, decode_responses=True, password='jimmy')
r = redis.Redis(connection_pool=pool)

#--vocab_file=third_party/ORB_SLAM3/Vocabulary/ORBvoc.txt \
#    --settings_file=third_party/ORB_SLAM3/Examples/RGB-D/TUM1.yaml \
#parser = argparse.ArgumentParser()
#parser.add_argument("--vocab_file", required=True)
#parser.add_argument("--settings_file", required=True)
#parser.add_argument("--dataset_path", required=True)
#args = parser.parse_args()

#img_files = sorted(glob(os.path.join(args.dataset_path, 'rgb/*.png')))
#vocab_file="/media/jimmy/01D9E5979B0C5780/ORB-SLAM3-python/third_party/ORB_SLAM3/Vocabulary/ORBvoc.txt"
#settings_file="/media/jimmy/01D9E5979B0C5780/ORB-SLAM3-python/third_party/ORB_SLAM3/Examples/RGB-D/TUM1.yaml"
#slam = orbslam3.system(vocab_file, settings_file, orbslam3.Sensor.RGBD)
#slam.set_use_viewer(True)
#slam.initialize()


app = Flask(__name__)
app.config['DEBUG'] = False

cors = CORS(app)
app.config['CORS_HEADERS'] = 'Content-Type'


y = 0







@app.route('/move/<string:direction>')
def move(direction,amount=0,custom_x=0,custom_y=0):
    global camera_x,camera_y,_movement_step,_feedback,hi
    if direction == "home":
        hi.movej_angle(0,0,0,25, 100, 0)
        hi.wait_stop()
        hi.get_scara_param()
        r.set("global_camera_xy",str(hi.x)+","+str(hi.y))
        return "home"

    elif direction == "custom":
        x = float(request.args.get('x', 0))
        y = float(request.args.get('y', 0))
        z = float(request.args.get('z', 0))
        r = float(request.args.get('r', 20))
        roughly = float(request.args.get('roughly', 0))
        lr = int(request.args.get('lr', 1))
        hi.get_scara_param()
        rett=hi.movel_xyz(hi.x+x,hi.y+y,hi.z+z,r,100)
        #custom?x=0&y=0&z=-10&r=0&roughly=0
        #res = hi.new_movej_xyz_lr(0,0,-100,0,100,0,1)
        hi.wait_stop()
        ret=f"ret={rett}: x = {hi.x+x} y = {hi.y+y} z = {hi.z+z} roughly = {roughly} "
        resp={'ret':ret}
        hi.get_scara_param()
        r.set("global_camera_xy",str(hi.x)+","+str(hi.y))
        return jsonify(resp)

    else:
        if "amount" in request.args:
            amount = float(request.args.get('amount', _movement_step))
        x=0
        y=0
        z=0

        if direction=="right":
            x=-amount
        elif direction=="left":
            x=amount
        elif direction=="up":
            y=amount
        elif direction=="down":
            y=-amount
        elif direction=="top":
            z=amount
        elif direction=="bottom":
            z=-amount
        hi.new_movej_xyz_lr(x,y,z,20,100,0,1)
        hi.wait_stop()
        ret=f"{direction} x = {x}y = {y}z = {z}"
        resp={'ret':ret}
        hi.get_scara_param()
        r.set("global_camera_xy",str(hi.x)+","+str(hi.y))
        return jsonify(resp)



@app.route('/scan')
def scan():
    global prev_value
    r.set("mode", "camera_ready")
    r.delete("detections")
          #while 1:
        #rett=hi.movel_xyz(hi.x,hi.y+50,hi.z,25,20)
        #hi.wait_stop()
    #    rett=hi.movel_xyz(hi.x,hi.y+50,hi.z,25,20)
    #    hi.wait_stop()
    #hi.get_scara_param()
        #r.set("global_camera_xy",str(hi.x)+","+str(hi.y))
    #r.set("mode", "camera_ready")
           #time.sleep(100)
       #rett=hi.movel_xyz(hi.x+50,hi.y-100,hi.z,25,20)
    return " camera_ready OK";


@app.route('/xbackward')
def xbackward():

    time.sleep(1)
    xy = r.get("global_camera_xy").split(",")
    print(xy)
    r.set("global_camera_xy", str(int(xy[0]) - 25) + "," + xy[1])
    return render_template('index.html');


@app.route('/xforward')
def xfarward():
    return render_template('index.html');




@app.route('/catch')
def catch():
    #video_dir.move_increase_y(10)
    print("catch")
    return render_template('index.html');


@app.route('/release')
def release():
    #video_dir.move_decrease_y(10)
    return render_template('index.html');
@app.route('/zup')
def zup():
    #video_dir.move_decrease_x(10)
    return render_template('index.html');
@app.route('/zdown')
def zdown():
    #video_dir.move_increase_x(10)
    return render_template('index.html');


#def autopick():
#    publish_result = mqtt_client.publish(topic, "/flask/scan")
#    return render_template('index.html');


@app.route('/zerosetting')
def zero():
    # publish_result = mqtt_client.publish(topic, "/flask/home")
    time.sleep(1)
    r.set("global_camera_xy", "0,0")
    return render_template('index.html');


@app.route('/home')
def home():
    # publish_result = mqtt_client.publish(topic, "/flask/home")
    time.sleep(1)
    r.set("global_camera_xy", "0,0")
    return render_template('index.html');



@app.route('/')
def index():
    return render_template('index.html')


@app.route('/msg')
def msg():
    return render_template('index.html')


@app.route('/update_mushroom_map' ,methods=['GET'])
def update_mushroom_map():
    detections=r.hgetall("detections")
    print(jsonify({"response":detections}))
    return jsonify({"response":detections})

def gen(camera):
    """Video streaming generator function."""
    yield b'--frame\r\n'
    while True:
        frame = camera.get_frame()
        yield b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n--frame\r\n'


@app.route('/video_feed')
def video_feed():
    """Video streaming route. Put this in the src attribute of an img tag."""
    return Response(gen(Camera()),
                    mimetype='multipart/x-mixed-replace; boundary=--frame')







#bounding_boxes=None
bboxes_msg=None
result_img_rgb=None
img_rgb=None
points_xyz_rgb=np.asarray([[]])
pointsxyzrgb_total=np.asarray([[0,0,0,0,0,0]])
global_points_xyz_rgb_list=np.asarray([()])
i=0
class yolox_ros(yolox_py):
    def __init__(self) -> None:
        raw_image_topic = '/camera/color/image_rect_raw'
        depth_image_topic = '/camera/depth/image_rect_raw'
        depth_info_topic = '/camera/depth/camera_info'
        move_x="/move/x"

        # ROS2 init
        super().__init__('yolox_ros', load_params=False)

        self.setting_yolox_exp()


        self.bridge = CvBridge()
        
        #self.pub = self.create_publisher(BoundingBoxes,"/yolox/bounding_boxes", 10)
        self.pub_bounding_boxes_cords = self.create_publisher(BoundingBoxesCords,"/yolox/bounding_boxes_cords", 1)
        self.pub_boxes_img = self.create_publisher(Image,"/yolox/boxes_image", 10)
        self.pub_rpi5_boxes_img = self.create_publisher(Image,"/yolox/rpi5/boxing_image", 10)
        self.pub_pointclouds = self.create_publisher(PointCloud2,'/yolox/pointclouds', 10)
        self.sub_depth_image = self.create_subscription(Image, depth_image_topic, self.imageDepthCallback, 1)
        self.sub_info = self.create_subscription(CameraInfo, depth_info_topic, self.imageDepthInfoCallback, 1)
        self.sub_move_xy_info = self.create_subscription(String, move_x, self.MoveXYZCallback, 1)
        self.sub_rpi_raw_img = self.create_subscription(Image,"/yolox/rpi5/raw_image",self.rpi5_imageflow_callback, 10)
        self.intrinsics = None
        self.pix = None
        self.pix_grade = None

        if (self.sensor_qos_mode):
            self.sub = self.create_subscription(Image,raw_image_topic,self.imageflow_callback, qos_profile_sensor_data)
        else:
            self.sub = self.create_subscription(Image,raw_image_topic,self.imageflow_callback, 10)

    
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
            model.load_state_dict(ckpt["model"])
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

    def extract_points(self,frame):
        logger.info("extract_points")

        orb = cv2.ORB_create()
        image = cv2.cvtColor(frame.image, cv2.COLOR_BGR2GRAY)
        # detection corners
        pts = cv2.goodFeaturesToTrack(image, 3000, qualityLevel=0.01, minDistance=3)
        #logger.info("pts: {}".format(pts ))
        # extract features
        kps = [cv2.KeyPoint(x=pt[0][0], y=pt[0][1], size=20) for pt in pts]
        logger.info("pts: {}".format(pts ))
        kps, des = orb.compute(image, kps)
        #logger.info("kps, des: {},{}".format(kps, des ))

        kps = np.array([(kp.pt[0], kp.pt[1]) for kp in kps])
        return kps, des

# 当前帧的角点和上一帧的进行配准
    def match_points(self,frame):
        #logger.info("match_points")
        bfmatch = cv2.BFMatcher(cv2.NORM_HAMMING)
        matches = bfmatch.knnMatch(frame.curr_des, frame.last_des, k=2)
        match_kps, idx1, idx2 = [], [], []

        for m,n in matches:
            if m.distance < 0.75*n.distance:
                idx1.append(m.queryIdx)
                idx2.append(m.trainIdx)

                p1 = frame.curr_kps[m.queryIdx]
                p2 = frame.last_kps[m.trainIdx]
                match_kps.append((p1, p2))
        assert len(match_kps) >= 8

        frame.curr_kps = frame.curr_kps[idx1]
        frame.last_kps = frame.last_kps[idx2]

        return match_kps
    def normalize(self,K, pts):
        Kinv = np.linalg.inv(K)
        # turn [[x,y]] -> [[x,y,1]]
        add_ones = lambda x: np.concatenate([x, np.ones((x.shape[0], 1))], axis=1)
        norm_pts = np.dot(Kinv, add_ones(pts).T).T[:, 0:2]
        return norm_pts
# 八点法对本质矩阵求解
    def fit_essential_matrix(self,match_kps):
        global K,frame
        match_kps = np.array(match_kps)

        # 使用相机内参对角点坐标归一化
        norm_curr_kps = self.normalize(K, match_kps[:, 0])
        norm_last_kps = self.normalize(K, match_kps[:, 1])

        # 求解本质矩阵和内点数据
        model, inliers = ransac((norm_last_kps, norm_curr_kps),
                                self.EssentialMatrixTransform,
                                min_samples=8,              # 最少需要 8 个点
                                residual_threshold=0.005,
                                max_trials=200)

        frame.curr_kps = frame.curr_kps[inliers]
        frame.last_kps = frame.last_kps[inliers]

        return model.params

    # 从本质矩阵中分解出相机运动 R、t
    def extract_Rt(E):
        W = np.mat([[0,-1,0],[1,0,0],[0,0,1]],dtype=float)
        U,d,Vt = np.linalg.svd(E)

        if np.linalg.det(U)  < 0: U  *= -1.0
        if np.linalg.det(Vt) < 0: Vt *= -1.0

        # 相机没有转弯，因此 R 的对角矩阵非常接近 diag([1,1,1])
        R = (np.dot(np.dot(U, W), Vt))
        if np.sum(R.diagonal()) < 0:
            R = np.dot(np.dot(U, W.T), Vt)

        t = U[:, 2]     # 相机一直向前，分量 t[2] > 0
        if t[2] < 0:
            t *= -1

        Rt = np.eye(4)
        Rt[:3, :3] = R
        Rt[:3, 3] = t
        return Rt          # Rt 为从相机坐标系的位姿变换到世界坐标系的位姿

    # opencv 的三角测量函数
    # def triangulate(pts1, pts2, pose1, pose2):
        # pts1 = normalize(pts1)
        # pts2 = normalize(pts2)

        # pose1 = np.linalg.inv(pose1)
        # pose2 = np.linalg.inv(pose2)

        # points4d = cv2.triangulatePoints(pose1[:3], pose2[:3], pts1.T, pts2.T).T
        # points4d /= points4d[:, 3:]
        # return points4d


    # 自己写的的三角测量函数
    def triangulate(self,pts1, pts2, pose1, pose2):
        global K
        pose1 = np.linalg.inv(pose1)            # 从世界坐标系变换到相机坐标系的位姿, 因此取逆
        pose2 = np.linalg.inv(pose2)

        pts1 = self.normalize(K, pts1)                 # 使用相机内参对角点坐标归一化
        pts2 = self.normalize(K, pts2)

        points4d = np.zeros((pts1.shape[0], 4))
        for i, (kp1, kp2) in enumerate(zip(pts1, pts2)):
            A = np.zeros((4,4))
            A[0] = kp1[0] * pose1[2] - pose1[0]
            A[1] = kp1[1] * pose1[2] - pose1[1]
            A[2] = kp2[0] * pose2[2] - pose2[0]
            A[3] = kp2[1] * pose2[2] - pose2[1]
            _, _, vt = np.linalg.svd(A)         # 对 A 进行奇异值分解
            points4d[i] = vt[3]

        points4d /= points4d[:, 3:]            # 归一化变换成齐次坐标 [x, y, z, 1]
        return points4d

    # 画出角点的运动轨迹
    def draw_points(self,frame):
        for kp1, kp2 in zip(frame.curr_kps, frame.last_kps):
            u1, v1 = int(kp1[0]), int(kp1[1])
            u2, v2 = int(kp2[0]), int(kp2[1])
            cv2.circle(frame.image, (u1, v1), color=(0,0,255), radius=3)
            cv2.line(frame.image, (u1, v1), (u2, v2), color=(255,0,0))
        return None

    # 筛选角点
    def check_points(self,points4d):
        # 判断3D点是否在两个摄像头前方
        good_points = points4d[:, 2] > 0
        # TODO: parallax、重投投影误差筛选等等 ....
        return good_points

    def process_frame(self,frame):
        global mapp
        # 提取当前帧的角点和描述子特征
        logger.info("---------------- process_frame----------------")
        frame.curr_kps, frame.curr_des = self.extract_points(frame)
        # 将角点位置和描述子通过类的属性传递给下一帧作为上一帧的角点信息
        Frame.last_kps, Frame.last_des = frame.curr_kps, frame.curr_des

        if frame.idx == 1:
            # 设置第一帧为初始帧，并以相机坐标系为世界坐标系
            frame.curr_pose = np.eye(4)
            points4d = [[0,0,0,1]]      # 原点为 [0, 0, 0] , 1 表示颜色
        else:
            # 角点配准, 此时会用 RANSAC 过滤掉一些噪声
            logger.info("---------------- match_points----------------")
            match_kps = self.match_points(frame)
            logger.info("frame: {}, curr_des: {}, last_des: {}, match_kps: {}".format(frame.idx, len(frame.curr_des), len(frame.last_des), len(match_kps)))
            # 使用八点法拟合出本质矩阵
            essential_matrix = self.fit_essential_matrix(match_kps)
            logger.info("---------------- Essential Matrix ----------------")
            logger.info(essential_matrix)
            # 利用本质矩阵分解出相机的位姿 Rt
            Rt = self.extract_Rt(essential_matrix)
            # 计算出当前帧相对于初始帧的相机位姿
            frame.curr_pose = np.dot(Rt, frame.last_pose)
            # 三角测量获得角点的深度信息
            points4d = self.triangulate(frame.last_kps, frame.curr_kps, frame.last_pose, frame.curr_pose)

            good_pt4d = self.check_points(points4d)
            points4d = points4d[good_pt4d]
            # TODO: g2o 后端优化
            self.draw_points(frame)
        mapp.add_observation(frame.curr_pose, points4d)     # 将当前的 pose 和点云放入地图中
        # 将当前帧的 pose 信息存储为下一帧的 last_pose 信息
        Frame.last_pose = frame.curr_pose
        return frame

    def rpi5_imageflow_callback(self,msg:Image) -> None:
            global bboxes_msg,result_img_rgb,img_rgb,mapp,frame
            img_rgb = self.bridge.imgmsg_to_cv2(msg,"bgr8")
            if img_rgb is not None:
                outputs, img_info = self.predictor.inference(img_rgb)
                #logger.info("outputs : {},".format((outputs)))

                try:
                    logger.info("mode={},mode==camera_ready,{}".format(r.get("mode"),r.get("mode")=="camera_ready"))
                    if  (outputs is not None) and r.get("scan")=="start" :#r.get("mode")=="camera_ready" and
                        #logger.info("output[0]{},img_info{}".format(outputs[0],img_info))
                        result_img_rgb, bboxes, scores, cls, cls_names,track_ids = self.predictor.visual(outputs[0], img_info)
                        if  bboxes is not None:
                            bboxes_msg = self.yolox2bboxes_msgs(bboxes, scores, cls, cls_names,track_ids, msg.header, img_rgb)

                    if result_img_rgb is not None:

                        #frame = Frame(result_img_rgb)
                        #frame = self.process_frame(frame)
                        #logger.info("process_frame")
                        #cv2.imshow("slam", frame.image)
                        #if cv2.waitKey(30) & 0xFF == ord('q'): 
                        #    exit()
                        #mapp.display()

                        img_rgb_pub = self.bridge.cv2_to_imgmsg(result_img_rgb,"bgr8")
                    else:
                        img_rgb_pub = self.bridge.cv2_to_imgmsg(img_rgb,"bgr8")

                    self.pub_rpi5_boxes_img.publish(img_rgb_pub)
                        #time.sleep(2)

                    #if (self.imshow_isshow):
                    #    cv2.imshow("YOLOX",result_img_rgb)
                    #    cv2.waitKey(1)
                except Exception as e:
                    logger.error(e)
                    pass
    def imageflow_callback(self,msg:Image) -> None:
            global bboxes_msg,result_img_rgb,img_rgb,mapp,frame
            img_rgb = self.bridge.imgmsg_to_cv2(msg,"bgr8")
            if img_rgb is not None:
                outputs, img_info = self.predictor.inference(img_rgb)
                #logger.info("outputs : {},".format((outputs)))

                try:
                    logger.info("mode={},mode==camera_ready,{}".format(r.get("mode"),r.get("mode")=="camera_ready"))
                    if  (outputs is not None) and r.get("scan")=="start" :#r.get("mode")=="camera_ready" and
                        #logger.info("output[0]{},img_info{}".format(outputs[0],img_info))
                        result_img_rgb, bboxes, scores, cls, cls_names,track_ids = self.predictor.visual(outputs[0], img_info)
                        if  bboxes is not None:
                            bboxes_msg = self.yolox2bboxes_msgs(bboxes, scores, cls, cls_names,track_ids, msg.header, img_rgb)

                    if result_img_rgb is not None:

                        #frame = Frame(result_img_rgb)
                        #frame = self.process_frame(frame)
                        #logger.info("process_frame")
                        #cv2.imshow("slam", frame.image)
                        #if cv2.waitKey(30) & 0xFF == ord('q'): 
                        #    exit()
                        #mapp.display()

                        img_rgb_pub = self.bridge.cv2_to_imgmsg(result_img_rgb,"bgr8")
                    else:
                        img_rgb_pub = self.bridge.cv2_to_imgmsg(img_rgb,"bgr8")

                    self.pub_boxes_img.publish(img_rgb_pub)
                        #time.sleep(2)

                    #if (self.imshow_isshow):
                    #    cv2.imshow("YOLOX",result_img_rgb)
                    #    cv2.waitKey(1)
                except Exception as e:
                    logger.error(e)
                    pass
    def my_pose_estimation(self,depth, rgb,box):
        global i,pointsxyzrgb_total
        logger.info("before convertion depth:{}".format(depth.shape))
        intrinsics = self.intrinsics
        depth = np.asanyarray(depth)  # * depth_scale # 1000 mm => 0.001 meters
        rgb = np.asanyarray(rgb)
        #rows,cols  = depth.shape
        rows=480#self.intrinsics.width 
        cols=848#self.intrinsics.height
        logger.info("depth:{}".format(depth.shape))
        i=i+1
        rotation_results=[]
        for box in bboxes_msg.bounding_boxes:
            if 1:# not r.hexists("detections",box.class_id):
                col, row = np.meshgrid(np.arange(cols), np.arange(rows), sparse=True)
                row = row.astype(float)
                col = col.astype(float)
                logger.info("r:{},c:{}".format(row.shape,col.shape))
                logger.info("box.xmin:{},box.xmax:{},box.ymin:{},box.ymax:{}".format(box.xmin,box.xmax,box.ymin,box.ymax))
                valid = (depth[box.ymin:box.ymax,box.xmin:box.xmax] > 0) #& (depth < clip_distance_max) #remove from the depth image all values above a given value (meters).
                valid = np.ravel(valid)
                z = depth[box.ymin:box.ymax,box.xmin:box.xmax]
                #z = depth[box.xmin:box.xmax,box.ymin:box.ymax] 
                logger.info("z:{}".format(z.shape))
                x =  z * (col[:,box.xmin:box.xmax] - intrinsics.ppx) / intrinsics.fx
                y =  z * (row[box.ymin:box.ymax,:] - intrinsics.ppy) / intrinsics.fy
                #logger.info("x,y:{},{}".format(x,y))
                z = np.ravel(z)[valid]
                x = np.ravel(x)[valid]
                y = np.ravel(y)[valid]

                # Stack x, y, z coordinates into a point array
                points = np.vstack((x, y, z)).T

                # Reshape the RGB image for color mapping and normalize colors
                #red = np.ravel(rgb[box.ymin:box.ymax,box.xmin:box.xmax,0])[valid]
                #green = np.ravel(rgb[box.ymin:box.ymax,box.xmin:box.xmax,1])[valid]
                #blue = np.ravel(rgb[box.ymin:box.ymax,box.xmin:box.xmax,2])[valid]
                #colors= np.dstack((red, green, blue))
                #pointsxyzrgb = pointsxyzrgb.reshape(-1,6)
                #colors = colors.reshape(-1, 3)
                #colors = (colors).astype(np.uint8)  # Ensure colors are uint8 for PyVista

                # Camera intrinsic parameters (adjust if known)
                #fx = fy = 382  # Focal length in pixels
                #cx = rows / 2.0
                #cy = cols / 2.0

                # Convert pixel coordinates to 3D coordinates in camera space
                #X = (x - cx) * z / fx
                #Y = (y - cy) * z / fy
                #Z = z

                # Stack into Nx3 array
                #points_3d = np.vstack((X, Y, Z)).T
                x_min = box.xmin#int(x_min)
                y_min = box.ymin# int(y_min)
                #width_bbox = int(width_bbox)
                #height_bbox = int(height_bbox)
                x_max = box.xmax# + width_bbox
                y_max =box.ymax # + height_bbox

                # Handle cases where the bounding box is out of image bounds
                if x_min < 0 or y_min < 0 or x_max > rows or y_max > cols:
                    print("Bounding box out of image bounds for box ID:", box.class_id)
                    continue

                # Get indices within the bounding box
                bbox_indices = ((x >= x_min) & (x < x_max) & (y >= y_min) & (y < y_max))

                # Get the points and colors within the bounding box
                #X_bbox = X[bbox_indices]
                #Y_bbox = Y[bbox_indices]
                #Z_bbox = Z[bbox_indices]
                points_bbox = points#np.vstack((X_bbox, Y_bbox, Z_bbox)).T
                #logger.info("in pose  points_bbox:{}".format(points_bbox))

                if points_bbox.shape[0] < 3:
                    logger.info("Not enough valid points for plane fitting for annotation ID:{}".format(box.class_id))
                    continue

                # Fit a plane to the 3D points
                # Compute the centroid
                centroid = np.mean(points_bbox, axis=0)
                # Center the points
                points_centered = points_bbox - centroid
                # Compute covariance matrix
                H_matrix = np.dot(points_centered.T, points_centered)
                # Singular Value Decomposition
                _, _, Vt = np.linalg.svd(H_matrix)
                # Normal vector is the last row of Vt
                normal_vector = Vt[-1, :]
                # Ensure normal vector points towards the camera (positive Z)
                if normal_vector[2] > 0:
                    normal_vector = -normal_vector

                # Compute rotation matrix
                z_axis = np.array([0, 0, 1])
                rotation_axis = np.cross(z_axis, normal_vector)
                rotation_axis_norm = np.linalg.norm(rotation_axis)
                if rotation_axis_norm < 1e-6:
                    # The normal vector is aligned with the z-axis
                    rotation_matrix = np.eye(3)
                else:
                    rotation_axis = rotation_axis / rotation_axis_norm
                    angle = np.arccos(np.clip(np.dot(z_axis, normal_vector), -1.0, 1.0))
                    # Rodrigues' rotation formula
                    K = np.array([[0, -rotation_axis[2], rotation_axis[1]],
                                    [rotation_axis[2], 0, -rotation_axis[0]],
                                    [-rotation_axis[1], rotation_axis[0], 0]])
                    rotation_matrix = np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * np.dot(K, K)

                # Convert rotation matrix to Euler angles (XYZ convention)
                r = R.from_matrix(rotation_matrix)
                euler_angles = r.as_euler('xyz', degrees=True)

                # Store rotation results
                rotation_results.append({
                    'annotation_id': 1,
                    'rotation_x': euler_angles[0],
                    'rotation_y': euler_angles[1],
                    'rotation_z': euler_angles[2],
                    'centroid': centroid,
                    'normal_vector': normal_vector
                })
                logger.info("Rotation{}\n,rotation_matrix{}".format(euler_angles,rotation_matrix))
                #time.sleep(1)
                
                
                #red = np.ravel(rgb[box.ymin:box.ymax,box.xmin:box.xmax,0])[valid]
                #green = np.ravel(rgb[box.ymin:box.ymax,box.xmin:box.xmax,1])[valid]
                #blue = np.ravel(rgb[box.ymin:box.ymax,box.xmin:box.xmax,2])[valid]
                #pointsxyzrgb = np.dstack((x, y, z, red, green, blue))
                #pointsxyzrgb = pointsxyzrgb.reshape(-1,6)
                #pointsxyzrgb_total=np.concatenate((pointsxyzrgb_total,pointsxyzrgb))
                #logger.info("pointsxyzrgb_total :{}".format(pointsxyzrgb_total))
                #logger.info("pointsxyzrgb_total:{},shape{}".format(pointsxyzrgb_total,pointsxyzrgb_total.shape))
        #self.create_point_cloud_file2(pointsxyzrgb_total,"new_room_total{}.ply".format(i))
        #i=i-1
        #pointsxyzrgb_total=np.delete(pointsxyzrgb_total,0)
        #return pointsxyzrgb_total
    def create_point_cloud_file2(self,vertices, filename):
        ply_header = '''ply
    format ascii 1.0
    element vertex %(vert_num)d
    property float x
    property float y
    property float z
    property uchar red
    property uchar green
    property uchar blue
    end_header
    '''
        with open(filename, 'w') as f:
            f.write(ply_header %dict(vert_num=len(vertices)))
            np.savetxt(f,vertices,'%f %f %f %d %d %d')
    def pose_estimation(self,depth,raw_img,bounding_boxes):
        print(" pose_estimation bounding_boxes:", bounding_boxes)
        # Convert depth image to proper format if necessary
        if depth.dtype != np.float32:
            depth = depth.astype(np.float32)
        # Convert depth units if necessary (e.g., scaling)
        # depth = depth * depth_scale  # Uncomment and set depth_scale if needed
        # Extract annotations for the specific image
        #image_id = get_image_id_by_name(image_name, coco)
        #annotations = [ann for ann in coco['annotations'] if ann['image_id'] == image_id]

        # Generate the 3D point cloud for the entire image using your provided code
        (height, width, _) = raw_img.shape
        xx, yy = np.meshgrid(np.arange(0, width), np.arange(0, height))

        # Flatten the arrays to create a point cloud
        x = xx.flatten()
        y = yy.flatten()
        z = depth.flatten()

        # Remove invalid depth points
        valid = (z > 0) & (~np.isnan(z))
        x = x[valid]
        y = y[valid]
        z = z[valid]

        # Stack x, y, z coordinates into a point array
        points = np.vstack((x, y, z)).T

        # Reshape the RGB image for color mapping and normalize colors
        colors = raw_img.reshape(-1, 3)[valid]
        colors = (colors).astype(np.uint8)  # Ensure colors are uint8 for PyVista

        # Camera intrinsic parameters (adjust if known)
        fx = fy = 382  # Focal length in pixels
        cx = width / 2.0
        cy = height / 2.0

        # Convert pixel coordinates to 3D coordinates in camera space
        X = (x - cx) * z / fx
        Y = (y - cy) * z / fy
        Z = z

        # Stack into Nx3 array
        points_3d = np.vstack((X, Y, Z)).T

        # Create a PyVista point cloud
        #cloud = pv.PolyData(points_3d)
        #cloud["RGB"] = colors

        # Create a PyVista plotter
        #plotter = pv.Plotter()

        # Add the point cloud to the plotter and set up color mapping
        #plotter.add_points(cloud, scalars="RGB", rgb=True, point_size=1)

        # Set plot labels
        #plotter.set_background("white")
        #plotter.add_axes()

        # Prepare to store rotation values and normals
        rotation_results = []
    
        # Process each mushroom instance
        for box in bounding_boxes:
            # Extract bounding box
            #bbox = ann['bbox']  # Format: [x_min, y_min, width, height]
            #x_min, y_min, width_bbox, height_bbox = bbox
            x_min = box.xmin#int(x_min)
            y_min = box.ymin# int(y_min)
            #width_bbox = int(width_bbox)
            #height_bbox = int(height_bbox)
            x_max = box.xmax# + width_bbox
            y_max =box.ymax # + height_bbox

            # Handle cases where the bounding box is out of image bounds
            if x_min < 0 or y_min < 0 or x_max > width or y_max > height:
                print("Bounding box out of image bounds for box ID:", box.class_id)
                continue

            # Get indices within the bounding box
            bbox_indices = ((x >= x_min) & (x < x_max) & (y >= y_min) & (y < y_max))

            # Get the points and colors within the bounding box
            X_bbox = X[bbox_indices]
            Y_bbox = Y[bbox_indices]
            Z_bbox = Z[bbox_indices]
            points_bbox = np.vstack((X_bbox, Y_bbox, Z_bbox)).T

            if points_bbox.shape[0] < 3:
                print("Not enough valid points for plane fitting for annotation ID:", box.class_id)
                continue

            # Fit a plane to the 3D points
            # Compute the centroid
            centroid = np.mean(points_bbox, axis=0)
            # Center the points
            points_centered = points_bbox - centroid
            # Compute covariance matrix
            H_matrix = np.dot(points_centered.T, points_centered)
            # Singular Value Decomposition
            _, _, Vt = np.linalg.svd(H_matrix)
            # Normal vector is the last row of Vt
            normal_vector = Vt[-1, :]
            # Ensure normal vector points towards the camera (positive Z)
            if normal_vector[2] > 0:
                normal_vector = -normal_vector

            # Compute rotation matrix
            z_axis = np.array([0, 0, 1])
            rotation_axis = np.cross(z_axis, normal_vector)
            rotation_axis_norm = np.linalg.norm(rotation_axis)
            if rotation_axis_norm < 1e-6:
                # The normal vector is aligned with the z-axis
                rotation_matrix = np.eye(3)
            else:
                rotation_axis = rotation_axis / rotation_axis_norm
                angle = np.arccos(np.clip(np.dot(z_axis, normal_vector), -1.0, 1.0))
                # Rodrigues' rotation formula
                K = np.array([[0, -rotation_axis[2], rotation_axis[1]],
                                [rotation_axis[2], 0, -rotation_axis[0]],
                                [-rotation_axis[1], rotation_axis[0], 0]])
                rotation_matrix = np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * np.dot(K, K)

            # Convert rotation matrix to Euler angles (XYZ convention)
            r = R.from_matrix(rotation_matrix)
            euler_angles = r.as_euler('xyz', degrees=True)

            # Store rotation results
            rotation_results.append({
                'annotation_id': 1,
                'rotation_x': euler_angles[0],
                'rotation_y': euler_angles[1],
                'rotation_z': euler_angles[2],
                'centroid': centroid,
                'normal_vector': normal_vector
            })
            print(f"Rotation: {euler_angles}")

        # Create a plane using the centroid and normal vector
        #plane_size = max(width_bbox, height_bbox) * depth.mean() / fx  # Adjust size as needed
        #plane = pv.Plane(center=centroid, direction=normal_vector, i_size=plane_size, j_size=plane_size)

        # Create an arrow to represent the normal vector
        #arrow_length = plane_size * 0.5  # Adjust length as needed
        #arrow = pv.Arrow(start=centroid, direction=normal_vector, scale="auto")

        # Add the plane and arrow to the plotter
        #plotter.add_mesh(plane, color='blue', opacity=0.5)
        #plotter.add_mesh(arrow, color='red')

        # Optionally, plot the points of the mushroom instance
        #mushroom_point_cloud = pv.PolyData(points_bbox)
        #plotter.add_mesh(mushroom_point_cloud, color='yellow', point_size=2, render_points_as_spheres=True)

        # Print rotation results (optional)

        #print(f"Annotation ID: {ann['id']}, Rotation: {euler_angles}")

        # Show the plot
        #plotter.show()
    def imageDepthCallback(self, data):
        global bboxes_msg,result_img_rgb,i,points_xyz_rgb,global_points_xyz_rgb_list

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
  
            # pick one pixel among all the pixels with the closest range:
            #indices = np.array(np.where(cv_image == cv_image[cv_image > 0].min()))[:,0]
            global_camera_xy=r.get("global_camera_xy")
            camera_xy=global_camera_xy.split(",")
            mode=r.get("mode")=="camera_ready"
            logger.info("in  imageDepthCallback camera_xy:{}, {}".format(camera_xy[0],camera_xy[1]))

            if bboxes_msg is not None and len(bboxes_msg.bounding_boxes)>0  and  self.intrinsics is not None:
                #self.pose_estimation(cv_image,result_img_rgb,bboxes_msg.bounding_boxes)
                if(result_img_rgb is not None):
                    self.my_pose_estimation(cv_image, result_img_rgb,bboxes_msg.bounding_boxes)
                    #points_xyz_rgb=np.delete(points_xyz_rgb,0,axis=0)
                    #logger.info("glabal xyz {}".format(points_xyz_rgb.shape))
                
                r.set("mode","pickup_ready")
                boxes_cords=BoundingBoxesCords()

                for box in bboxes_msg.bounding_boxes:
                    box_cord=BoundingBoxCord()
                    #logger.info("probability,{},pixal x={},y={}".format(box.probability,box.class_id,(box.xmin+box.xmax)/2,(box.ymin+box.ymax)/2))
                    line ='probability:%4.2f,track_id:%s'%(box.probability,box.class_id)
                    #pix = (indices[1], indices[0])
                    pix = (int((box.xmin+box.xmax)/2),int((box.ymin+box.ymax)/2))
                    self.pix = pix
                    #line += '\tDepth at pixel(%3d, %3d): %7.1f(mm).' % (pix[0], pix[1], cv_image[pix[1], pix[0]])
                    if self.intrinsics and pix[0] <848 and pix[1]<480:
                        depth = cv_image[pix[1], pix[0]]
                        result = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [pix[0], pix[1]], depth)
                        line += '  local Coordinate: %8.2f %8.2f %8.2f.' % (result[0], result[1], result[2])
                    else:
                        continue
                    if (not self.pix_grade is None):
                        line += ' Grade: %2d' % self.pix_grade
                    line += '\r'

                    #logger.info("detections id:{},if exist {}".format(box.class_id,r.hexists("detections",str(box.class_id))))
                    #x=(int(float(result[0])))            #Arm#-------> Y    #camera   <--- #Y
                    #y=-(int(float(result[1])))               #|              #|
                    #y-=170                                  #\|/      X      #\|/         X               
                     
                    x=(int(float(result[0])))            #Arm#-------> Y    #camera   --- >#Y
                    y=-(int(float(result[1])))               #|              #/|\
                    y-=270                                 #\|/      X       #|         X   
                    # 
                    #x=(int(float(result[1])))           #Arm#-------> Y    #camera   --- >#Y
                    #y=(int(float(result[0])))               #|              #   \
                    #y-=90                                 #\|/      X       #  \|/        X                   
                    obj=str(int(float(camera_xy[0]))+x)+","+str(int(float(camera_xy[1]))+y)+","+str(int(float(result[2])))
                    #logger.info(line)
                    if not r.hexists("detections",str(box.class_id)):
                        r.hset("detections", box.class_id, obj)
                        r.lpush("queue",box.class_id)
                        r.hset("detections_history", box.class_id, obj) 
                        box_cord.x=int(float(camera_xy[0]))+x
                        box_cord.y=int(float(camera_xy[1]))+y
                        box_cord.class_id= box.class_id                      
                        boxes_cords.bounding_boxes.append(box_cord)
                    #else:
                    #     r.hdel("detections", box.class_id)

                r.set("mode","camera_ready")
                #gray=((points_xyz_rgb[:,3])+(points_xyz_rgb[:,4]) + (points_xyz_rgb[:,5]))/3
                if 0:# points_xyz_rgb.shape[0]>1:
                    points_xyz_rgb=points_xyz_rgb.T
                    new_points_xyz_rgb=list(zip(points_xyz_rgb[0],points_xyz_rgb[1],points_xyz_rgb[2],points_xyz_rgb[3]))
                    new_points_xyz_rgb_list=np.array(new_points_xyz_rgb,  dtype=[
                                                                                ('x', np.float32),
                                                                                ('y', np.float32),
                                                                                ('z', np.float32),
                                                                            ('i', np.uint8),
                                                                            # ('g', np.uint8),
                                                                            # ('b', np.uint8)]
                                                                                ])

                    #logger.info("new_points_xyz_rgb_list{}".format(new_points_xyz_rgb_list))
                    #global_points_xyz_rgb_list=np.concatenate((global_points_xyz_rgb_list,new_points_xyz_rgb_list))
                    #logger.info("global_points_xyz_rgb_list{}".format(global_points_xyz_rgb_list))

                    cloud_msg=rnp.msgify(PointCloud2, new_points_xyz_rgb_list)
                    cloud_msg.header=Header()
                    cloud_msg.header.stamp = self.get_clock().now().to_msg()#seconds #rospy.Time(t_us/10000000.0)
                    cloud_msg.header.frame_id = "/camera_link"
                    #self.pub_pointclouds.publish(cloud_msg)


                logger.info(boxes_cords)
                self.pub_bounding_boxes_cords.publish(boxes_cords)
                bboxes_msg=None

        except CvBridgeError as e:
            print(e)
            return
        except ValueError as e:
            return
        
    def MoveXYZCallback(self, data):
        global bboxes_msg
        logger.info("/move/x {}".format(data.data))
        xyz=data.data.split(";")
        x=xyz[0]
        y=xyz[1]
        z=xyz[2]
        logger.info(" local cord  to camera:{}, {}".format(x,y))
        try:

            global_camera_xy=r.get("global_camera_xy")
            camera_xy=global_camera_xy.split(",")
            logger.info("camera_xy:{}, {}".format(camera_xy[0],camera_xy[1]))
            if r.get("mode")=="camera_ready":
                #r.set("mode","pickup_ready")                            
                    obj=str(int(float(camera_xy[0]))+x)+","+str(int(float(camera_xy[1]))+y)+","+str(int(float(z)))
                    #logger.info(line)
                    if 1:
                        r.hset("detections", "-1", obj)
                        r.lpush("queue","-1")
                        r.hset("detections_history", "-1", obj) 
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
    def boxes_callback(self, data):
        logger.info("boxes_callback")
        #global bounding_boxes
        #bounding_boxes=None
        #if 1:#r.get("mode")=="camera_ready":
         #   bounding_boxes=data.bounding_boxes
            #r.set("mode","pickup_ready")
        #for box in bounding_boxes:
        #    logger.info(" boxes_callback probability,%4.2f,%s,x=%4.2f,y=%4.2f",box.probability,box.class_id,(box.xmin+box.xmax)/2,(box.ymin+box.ymax)/2)


def ros_main(args = None):
            rclpy.init(args=args)
            ros_class = yolox_ros()

            try:
                rclpy.spin(ros_class)
            except KeyboardInterrupt:
                pass
            finally:
                ros_class.destroy_node()
                rclpy.shutdown()

if __name__ == '__main__':
    ros_main()
    app.run(host='0.0.0.0', threaded=True,port='5001')

