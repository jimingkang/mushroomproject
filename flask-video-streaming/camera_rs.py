import json
import math
import os
import pickle
import  random
import cv2
import redis
import yaml
from flask import Flask
from flask_mqtt import Mqtt

from Mushroom import Mushroom
from base_camera import BaseCamera
#import pyrealsense2.pyrealsense2 as rs
import pyrealsense2 as rs

import os
import time
import numpy as np
import cv2
from threading import Event
from paho.mqtt import client as mqtt_client
import random
#from tracker import Tracker
import move_publish
import img_publish
from utils.torch_utils import select_device, load_classifier, time_sync
from utils.general import (
    check_img_size, non_max_suppression, apply_classifier, scale_coords,
    xyxy2xywh, strip_optimizer, set_logging)
from utils.datasets import LoadStreams, LoadImages, letterbox
from models.experimental import attempt_load
import torch.backends.cudnn as cudnn
import torch
#torch.cuda.is_available()
pipeline = rs.pipeline()  # 定义流程pipeline
config = rs.config()  # 定义配置config
config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 30)
profile = pipeline.start(config)  # 流程开始
align_to = rs.stream.color  # 与color流对齐
align = rs.align(align_to)
colors = [(random.randint(0, 255), random.randint(0, 255), random.randint(0, 255)) for j in range(100)]
detection_threshold = 0.5


broker=''
redis_server=''
try:
    for line in open("../ip.txt"):
        if line[0:6] == "broker":
            broker = line[9:len(line)]
        if line[0:5] == "redis":
            redis_server=line[9:len(line)]
except:
    pass
print(broker+" "+redis_server)
pool = redis.ConnectionPool(host='192.168.254.26', port=6379, decode_responses=True,password='jimmy')
r = redis.Redis(connection_pool=pool)

app = Flask(__name__)
app.config['MQTT_BROKER_URL'] = broker
#app.config['MQTT_BROKER_URL'] =  '192.168.254.43'
#app.config['MQTT_BROKER_URL'] = '10.0.0.18'
app.config['MQTT_BROKER_PORT'] = 1883
app.config['MQTT_USERNAME'] = ''  # Set this item when you need to verify username and password
app.config['MQTT_PASSWORD'] = ''  # Set this item when you need to verify username and password
app.config['MQTT_KEEPALIVE'] = 5  # Set KeepAlive time in seconds
app.config['MQTT_TLS_ENABLED'] = False  # If your server supports TLS, set it True
#socketio = SocketIO(app)
topic = '/flask/mqtt'
topic2 = '/flask/xyz'
topic3 = '/flask/serial'

app = Flask(__name__)
mqtt_client = Mqtt(app)

camera_xyz_list = []

def send_wake_up(ser):
    # Wake up
    # Hit enter a few times to wake the Printrbot
    ser.write(str.encode("\r\n\r\n"))
    time.sleep(2)   # Wait for Printrbot to initialize
    ser.flushInput()  # Flush startup text in serial input

def wait_for_movement_completion(ser,cleaned_line):

    Event().wait(1)
    if cleaned_line != '$X' or '$$':
        idle_counter = 0
        while True:
            # Event().wait(0.01)
            ser.reset_input_buffer()
            command = str.encode('?' + '\n')
            ser.write(command)
            grbl_out = ser.readline()
            grbl_response = grbl_out.strip().decode('utf-8')
            if grbl_response != 'ok':
                if grbl_response.find('Idle') > 0:
                    idle_counter += 1
            if idle_counter > 10:
                break
    return

def command(ser, command):
    send_wake_up(ser)
    if command:  # checks if string is empty
        print("Sending gcode:" + str(command))
        # converts string to byte encoded string and append newline
        command = str.encode(command + '\r\n')
        ser.write(command)  # Send g-code
        wait_for_movement_completion(ser, command)
        grbl_out = ser.readline()  # Wait for response with carriage return
        print(" : ", grbl_out.strip().decode('utf-8'))
    return grbl_out.strip().decode('utf-8')


#def command(ser, command):
#  ser.write(str.encode(command)) 
#  time.sleep(1)

def get_aligned_images():
    frames = pipeline.wait_for_frames()  # 等待获取图像帧
    #motion_data = frames.as_motion_frame().get_motion_data()
    #print("motion:"+motion_data)
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

class YoloV5:
    def __init__(self, yolov5_yaml_path='config/yolov5s.yaml'):
        '''初始化'''
        # 载入配置文件
        with open(yolov5_yaml_path, 'r', encoding='utf-8') as f:
            self.yolov5 = yaml.load(f.read(), Loader=yaml.SafeLoader)
        # 随机生成每个类别的颜色
        self.colors = [[np.random.randint(0, 255) for _ in range(
            3)] for class_id in range(self.yolov5['class_num'])]
        # 模型初始化
        self.init_model()

    @torch.no_grad()
    def init_model(self):
        '''模型初始化'''
        # 设置日志输出
        set_logging()
        # 选择计算设备
        device =select_device(self.yolov5['device'])
        # 如果是GPU则使用半精度浮点数 F16
        is_half = device.type != 'cpu'
        # 载入模型
        model = attempt_load(
            self.yolov5['weight'], map_location=device)  # 载入全精度浮点数的模型
        input_size = check_img_size(
            self.yolov5['input_size'], s=model.stride.max())  # 检查模型的尺寸
        if is_half:
            model.half()  # 将模型转换为半精度
        # 设置BenchMark，加速固定图像的尺寸的推理
        cudnn.benchmark = True  # set True to speed up constant image size inference
        # 图像缓冲区初始化
        img_torch = torch.zeros(
            (1, 3, self.yolov5['input_size'], self.yolov5['input_size']), device=device)  # init img
        # 创建模型
        # run once
        _ = model(img_torch.half()
                  if is_half else img) if device.type != 'cpu' else None
        self.is_half = is_half  # 是否开启半精度
        self.device = device  # 计算设备
        self.model = model  # Yolov5模型
        self.img_torch = img_torch  # 图像缓冲区

    def preprocessing(self, img):
        '''图像预处理'''
        # 图像缩放
        # 注: auto一定要设置为False -> 图像的宽高不同
        img_resize = letterbox(img, new_shape=(
            self.yolov5['input_size'], self.yolov5['input_size']), auto=False)[0]
        # print("img resize shape: {}".format(img_resize.shape))
        # 增加一个维度
        img_arr = np.stack([img_resize], 0)
        # 图像转换 (Convert) BGR格式转换为RGB
        # 转换为 bs x 3 x 416 x
        # 0(图像i), 1(row行), 2(列), 3(RGB三通道)
        # ---> 0, 3, 1, 2
        # BGR to RGB, to bsx3x416x416
        img_arr = img_arr[:, :, :, ::-1].transpose(0, 3, 1, 2)
        # 数值归一化
        # img_arr =  img_arr.astype(np.float32) / 255.0
        # 将数组在内存的存放地址变成连续的(一维)， 行优先
        # 将一个内存不连续存储的数组转换为内存连续存储的数组，使得运行速度更快
        # https://zhuanlan.zhihu.com/p/59767914
        img_arr = np.ascontiguousarray(img_arr)
        return img_arr

    @torch.no_grad()
    def detect(self, img, canvas=None, view_img=True):
        '''模型预测'''
        # 图像预处理
        img_resize = self.preprocessing(img)  # 图像缩放
        self.img_torch = torch.from_numpy(img_resize).to(self.device)  # 图像格式转换
        self.img_torch = self.img_torch.half(
        ) if self.is_half else self.img_torch.float()  # 格式转换 uint8-> 浮点数
        #print("img torch half")
        self.img_torch /= 255.0  # 图像归一化
        if self.img_torch.ndimension() == 3:
            self.img_torch = self.img_torch.unsqueeze(0)
        # 模型推理
        t1 = time_sync()
        pred = self.model(self.img_torch, augment=False)[0]
        # pred = self.model_trt(self.img_torch, augment=False)[0]
        # NMS 非极大值抑制
        pred = non_max_suppression(pred, self.yolov5['threshold']['confidence'],
                                   self.yolov5['threshold']['iou'], classes=None, agnostic=False)
        t2 = time_sync()
        # print("推理时间: inference period = {}".format(t2 - t1))
        # 获取检测结果
        det = pred[0]
        gain_whwh = torch.tensor(img.shape)[[1, 0, 1, 0]]  # [w, h, w, h]

        if view_img and canvas is None:
            canvas = np.copy(img)
        xyxy_list = []
        conf_list = []
        class_id_list = []
        if det is not None and len(det):
            # 画面中存在目标对象
            # 将坐标信息恢复到原始图像的尺寸
            det[:, :4] = scale_coords(
                img_resize.shape[2:], det[:, :4], img.shape).round()
            for *xyxy, conf, class_id in reversed(det):
                class_id = int(class_id)
                detections = []
                xyxy_list.append(xyxy)
                conf_list.append(conf)
                class_id_list.append(class_id)
                #if conf > detection_threshold:
                #detections.append([int(xyxy[0]), int(xyxy[1]), int(xyxy[2]), int(xyxy[3]), conf])
                #print(len(detections))
                #if len(detections)>0:
                    #tracker.update(canvas, detections)
                #   print("\n")
                    #for track in tracker.tracks:
                #        bbox = track.bbox
                #        x1, y1, x2, y2 = bbox
                #        track_id = track.track_id
                        #class_id = track.track_id
                        #self.colors[class_id]=colors[track_id]
                #        cv2.rectangle(canvas, (int(xyxy[0]), int(xyxy[1])), (int(xyxy[2]), int(xyxy[3])), (colors[track_id % len(colors)]), 3)
                if view_img:
                    # 绘制矩形框与标签
                    label = '%s %.2f' % (
                        self.yolov5['class_name'][class_id], conf)
                    self.plot_one_box(xyxy, canvas, label=label, color=self.colors[class_id], line_thickness=3)
        return canvas, class_id_list, xyxy_list, conf_list


    def plot_one_box(self, x, img, color=None, label=None, line_thickness=None):
        ''''绘制矩形框+标签'''
        tl = line_thickness or round(
            0.002 * (img.shape[0] + img.shape[1]) / 2) + 1  # line/font thickness
        color = color or [random.randint(0, 255) for _ in range(3)]
        c1, c2 = (int(x[0]), int(x[1])), (int(x[2]), int(x[3]))
        cv2.rectangle(img, c1, c2, color, thickness=tl, lineType=cv2.LINE_AA)
        if label:
            tf = max(tl - 1, 1)  # font thickness
            t_size = cv2.getTextSize(
                label, 0, fontScale=tl / 3, thickness=tf)[0]
            c2 = c1[0] + t_size[0], c1[1] - t_size[1] - 3
            cv2.rectangle(img, c1, c2, color, -1, cv2.LINE_AA)  # filled
            cv2.putText(img, label, (c1[0], c1[1] - 2), 0, tl / 3,
                        [225, 255, 255], thickness=tf, lineType=cv2.LINE_AA)



#model = YoloV5(yolov5_yaml_path='config/yolov5s.yaml')

global intr, depth_intrin, color_image, depth_image, aligned_depth_frame 
global track_id
g_xyz=''
class Camera(BaseCamera):
    video_source = 0
    def __init__(self):
        print("[INFO] 完成YoloV5模型加载")
        #model = YoloV5(yolov5_yaml_path='config/yolov5s.yaml')
        if os.environ.get('OPENCV_CAMERA_SOURCE'):
            Camera.set_video_source(int(os.environ['OPENCV_CAMERA_SOURCE']))
        super(Camera, self).__init__()
        #self.model=model
        #run()

    @staticmethod
    def set_video_source(source):
        Camera.video_source = source

    @mqtt_client.on_connect()
    def handle_connect(client, userdata, flags, rc):
        if rc == 0:
            print('Connected successfully')
            mqtt_client.subscribe(topic2)  # subscribe topic
        else:
            print('Bad connection. Code:', rc)

    @mqtt_client.on_message()
    def handle_mqtt_message(client, userdata, message):
        #print('Received message on topic:')
        #print(message.topic)
        if message.topic == topic2:
            data = dict(
                topic=message.topic,
                payload=message.payload.decode()
            )
            #print('Received message on topic: {topic} with payload: {payload}'.format(**data))
            xyz = data['payload']
            print("payload=" + xyz)
            xyz = xyz.split(";");
            if xyz:
                global g_xyz
                #xyz = xyz.split(";")
                #if g_xyz==xyz:
                #    return
                g_xyz=xyz
                x=xyz[0]
                if x :
                #for x in xyz:
                    first_xyz = x.split(",");
                    print(first_xyz)
                    #camera_xyz_list.append([float(first_xyz[0]),float(first_xyz[1]),int(first_xyz[2])])
                    track_id=int(first_xyz[2])
                    #print("G21 G91 G1 X" + str(int(float(first_xyz[0]) * 100)) + " F2540\r\n")
                    #if track_id == int(first_xyz[2]):
                    dis = aligned_depth_frame.get_distance(int(first_xyz[0]), int(first_xyz[1]))
                    camera_xyz = rs.rs2_deproject_pixel_to_point(depth_intrin, (int(first_xyz[0]), int(first_xyz[1])), dis)  # ????????xyz
                    print(camera_xyz)
                    camera_x=int(float(camera_xyz[0])*1000)
                    camera_y=int(float(camera_xyz[1])*1000)

                    global_camera_xy=r.exists("global_camera_xy")
                    if global_camera_xy==True:
                        gxy=r.get("global_camera_xy").split(",")
                        new_camera_x=camera_x+int(float(gxy[0]))
                        new_camera_y=camera_y+int(float(gxy[1]))

                        #distance = int(math.sqrt(old_camera_x * old_camera_x + camera_y * camera_y))
                        detected_x=r.zrangebyscore("detections_index_x",min=new_camera_x-5,max=new_camera_x+5)
                        detected_y=r.zrangebyscore("detections_index_y",min=new_camera_y-5,max=new_camera_y+5)

                        #for mush in detected:
                        #    obj = mush#pickle.loads(tmp[0]) #json.loads(Mushroom,tmp)
                        #    oldobj=obj.split(",")
                        #    print(obj)
                        #    if abs(old_camera_x-int(float(oldobj[0])))<5:
                        #        obj=oldobj[2]+","+str(distance)+","+str(old_camera_x)+","+str(camera_y)
                        #        r.zremrangebyscore("detections_index",min=distance,max=distance+5)
                        #        #r.zadd("detections_index", {obj: distance})
                        #        break
                        #else:
                        camera_x = camera_x +int(float(global_camera_xy))
                        distance = int(math.sqrt(new_camera_x * new_camera_x+ new_camera_y*new_camera_y))
                        if not r.hexists("detections",str(track_id)) and (len(detected_x)<=0 and len(detected_y)<=0):
                            print("distance:" + str(distance))  # +"camera_x:"+int(float(camera_x)))
                            r.zadd("detections_index_x", {str(track_id)+"_"+str(new_camera_x): new_camera_x})
                            r.zadd("detections_index_y", {str(track_id)+"_"+str(new_camera_y): new_camera_y})
                            r.hmset("detections", {str(track_id): str(new_camera_x) + "," + str(new_camera_y) + str(distance) + "," + str(track_id)})
                    else:
                        r.set("global_camera_xy","0,0")
                    x=1*float(camera_xyz[0]) * 1000
                    if abs(x)> 5:
                        #move_x=str(x) + " F100\r\n"
                        #cmd="G21 G91 G1 X" +move_x
                        #print(camera_xyz)
                        move_publish.run(topic3,str(camera_xyz[0])+","+str(track_id))
                        #command(ser,cmd)
                        #print(cmd)
                # socketio.emit('mqtt_message', data=data)
    @staticmethod
    def frames():
        #camera=cv2.VideoCapture(0)
        #camera = cv2.VideoCapture(Camera.video_source)
        #if not camera.isOpened():
        #    raise RuntimeError('Could not start camera.')
        try:
            while True:
                t_start = time.time()  
                # time.sleep(1000)
                # Wait for a coherent pair of frames: depth and color
                global intr, depth_intrin, color_image, depth_image, aligned_depth_frame 
                intr, depth_intrin, color_image, depth_image, aligned_depth_frame = get_aligned_images()  # 获取对齐的图像与相机内参
                if not depth_image.any() or not color_image.any():
                    continue
                # Convert images to numpy arrays
                # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
                depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(
                    depth_image, alpha=0.03), cv2.COLORMAP_JET)
                # Stack both images horizontally
                images = np.hstack((color_image, depth_colormap))

                # Show images

                #canvas, class_id_list, xyxy_list, conf_list = model.detect(color_image)

                t_end = time.time()  # 结束计时\
                # canvas = np.hstack((canvas, depth_colormap))
                # print(class_id_list)

                global camera_xyz_list 
                if camera_xyz_list:#xyxy_list:
                    for i in range(len(camera_xyz_list)):
                        ux =int(camera_xyz_list[i][0])# int((camera_xyz_list[i][0] + camera_xyz_list[i][2]) / 2)  # 计算像素坐标系的x
                        uy = int(camera_xyz_list[i][1]) #int((camera_xyz_list[i][1] + camera_xyz_list[i][3]) / 2)  # 计算像素坐标系的y
                        dis = aligned_depth_frame.get_distance(ux, uy)
                        camera_xyz = rs.rs2_deproject_pixel_to_point(depth_intrin, (ux, uy), dis)  # 计算相机坐标系的xyz
                        camera_xyz = np.round(np.array(camera_xyz), 3)  # 转成3位小数
                        camera_xyz = camera_xyz.tolist()
                        cv2.circle(color_image, (ux, uy), 4, (255, 255, 255), 5)  # 标出中心点
                        cv2.putText(color_image, str(camera_xyz) + '(m)', (ux + 20, uy + 10), 0, 1,
                                    [225, 255, 255], thickness=2, lineType=cv2.LINE_AA)  # 标出坐标
                camera_xyz_list = []
                fps = int(1.0 / (t_end - t_start))
                yield cv2.imencode('.jpg', color_image)[1].tobytes()
        finally:

            # Stop streaming
            pipeline.stop()
            #yield cv2.imencode('.jpg', canvas)[1].tobytes()


