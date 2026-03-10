import datetime
import os
import re
import subprocess
import time

import numpy as np

from io import BytesIO
import rclpy
import signal
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image

from PIL import Image as PILImage
import threading
import pybullet as p
import pybullet_data
from flask import send_file
import redis
from flask import Flask, json, render_template, Response, jsonify
from threading import Event
from flask import Flask, render_template, request, jsonify
from flask_mqtt import Mqtt
from sensor_msgs.msg import JointState
from cv_bridge import CvBridge,CvBridgeError
from flask import Flask, jsonify, request
from flask_cors import CORS
import subprocess
import cv2
frame=None
frame2=None
renderer=None
latest_frame = None
frame_lock = threading.Lock()
# --- Configuration ---
URDF_PATH = "/home/jimmy/Downloads/mushroomproject/harvest_ws/api_server/urdf/2160N0_urdf.urdf"  # Replace with your actual URDF file path
# --- Shared State (Thread Safe) ---
class RobotState:
    def __init__(self):
        self.lock = threading.Lock()
        self.joint_positions = {} # { "joint_name": position_float }

    def update(self, names, positions):
        with self.lock:
            for name, pos in zip(names, positions):
                self.joint_positions[name] = pos

    def get_snapshot(self):
        with self.lock:
            return self.joint_positions.copy()

shared_state = RobotState()




# --- PyBullet Renderer (Headless) ---
class URDFRenderer:
    def __init__(self, model_path):
        # Connect in DIRECT mode (no GUI window, suitable for servers)
        self.client_id = p.connect(p.DIRECT)
        
        # Add search path for standard URDFs (like plane.urdf)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        
        # Load the robot
        # useFixedBase=True ensures the robot doesn't fall over due to gravity
        self.robot_id = p.loadURDF(model_path, useFixedBase=True)
        
        # Map joint names to PyBullet indices
        self.joint_map = {}
        num_joints = p.getNumJoints(self.robot_id)
        for i in range(num_joints):
            info = p.getJointInfo(self.robot_id, i)
            # info[1] is the joint name (bytes)
            joint_name = info[1].decode('utf-8')
            self.joint_map[joint_name] = i

    def render(self, joint_state_dict):
        # 1. Update robot configuration
        for name, angle in joint_state_dict.items():
            if name in self.joint_map:
                joint_index = self.joint_map[name]
                p.resetJointState(self.robot_id, joint_index, angle)
        p.stepSimulation()
        # 2. Setup Camera (Position, Target, Up-vector)
        view_matrix = p.computeViewMatrix(
            cameraEyePosition=[-1.0, -1.0, 1.5], # Adjust these to frame your robot
            cameraTargetPosition=[0, 0, 0],
            cameraUpVector=[0, 0, 1]
        )
        projection_matrix = p.computeProjectionMatrixFOV(
            fov=60, aspect=1.0, nearVal=0.1, farVal=100.0
        )

        # 3. Render Image
        width, height, rgb_img, depth_img, seg_img = p.getCameraImage(
            width=1280,
            height=760,
            viewMatrix=view_matrix,
            projectionMatrix=projection_matrix,
            renderer=p.ER_BULLET_HARDWARE_OPENGL # or ER_TINY_RENDERER if no GPU
        )

        # 4. Convert to PIL Image
        # rgb_img is (width, height, 4) array [r,g,b,alpha]
        img = PILImage.fromarray(np.array(rgb_img), 'RGBA')
        return img
    def render2(self, joint_state_dict):

        for name, angle in joint_state_dict.items():
            if name in self.joint_map:
                p.resetJointState(
                    self.robot_id,
                    self.joint_map[name],
                    angle
                )

        p.stepSimulation()

        w, h, rgb, _, _ = p.getCameraImage(
            1280,
            760,
            self.view_matrix,
            self.projection_matrix,
            renderer=p.ER_BULLET_HARDWARE_OPENGL
        )

        img = PILImage.fromarray(np.array(rgb)[:, :, :3])
        return img



class MovePublisher(Node):
    def __init__(self):
        global renderer
        super().__init__('test_publisher')
        self.publisher = self.create_publisher(String, '/move', 10)
        self.subscription = self.create_subscription(Image,'/d405/yolox/boxes_image',self.chatter_callback,10)
        self.subscription2 = self.create_subscription(Image,'/d435/yolox/boxes_image',self.chatter_callback2,10)
        self.latest_message = None
        self.bridge = CvBridge()
        self.renderer = URDFRenderer(URDF_PATH)
        self.JointState_subscription = self.create_subscription(JointState,'/joint_states',self.JointState_callback,10)
        self.timer = self.create_timer(0.2, self.render_timer)
    def render_timer(self):
        joints = shared_state.get_snapshot()
        if not joints:
            return

        img = self.renderer.render(joints)

        global latest_frame
        with frame_lock:
            latest_frame = img



    def JointState_callback(self, msg):
        # Update the shared state with latest joint info
        #self.get_logger().info(f"Received JointState: {msg.name} positions: {msg.position}")
        shared_state.update(msg.name, msg.position)

    def chatter_callback(self, msg):
        global frame
        #print(f'chatter cb received: {msg.data}')
        frame = self.bridge.imgmsg_to_cv2(msg,"bgr8")
        #self.latest_message = msg.data
        #frame = msg.data
    def chatter_callback2(self, msg):
        global frame2
        #print(f'chatter cb received: {msg.data}')
        frame2 = self.bridge.imgmsg_to_cv2(msg,"bgr8")
        #self.latest_message = msg.data
        #frame = msg.data

    def publish_message(self,msg):
        self.publisher.publish(msg)


def ros2_thread(node):
    print('entering ros2 thread')
    rclpy.spin(node)
    print('leaving ros2 thread')


def sigint_handler(signal, frame):
    """
    SIGINT handler
    We have to know when to tell rclpy to shut down, because
    it's in a child thread which would stall the main thread
    shutdown sequence. So we use this handler to call
    rclpy.shutdown() and then call the previously-installed
    SIGINT handler for Flask
    """
    rclpy.shutdown()
    if prev_sigint_handler is not None:
        prev_sigint_handler(signal)


rclpy.init(args=None)
ros2_node = MovePublisher()
app = Flask(__name__)

CORS(app)  # Enable CORS
#CORS(app, resources={r"/*": {"origins": "*"}}) # 允许所有来源
threading.Thread(target=ros2_thread, args=[ros2_node]).start()
prev_sigint_handler = signal.signal(signal.SIGINT, sigint_handler)

@app.route('/')
def index():
    return render_template('index.html')



@app.route('/move/<string:direction>')  #move/custom?y=-100      #move/custom?x=0&y=0&z=-10&r=0&roughly=0
def move(direction,amount=0,custom_x=0,custom_y=0):
    global camera_x,camera_y,_movement_step,_feedback,hi
    if direction == "home":
        return "home"
    elif direction == "custom":
        x = float(request.args.get('x', 0))
        y = float(request.args.get('y', 0))
        z = float(request.args.get('z', 0))
        r = float(request.args.get('r', 20))
        roughly = float(request.args.get('roughly', 0))
        lr = int(request.args.get('lr', 1))
        ret=f"x={x};y={y};z={z};roughly={roughly}"
        msg = String()
        msg.data =ret
        ros2_node.publish_message(msg)
        resp={'ret':ret}
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
        ret=f"{direction} x = {x}y = {y}z = {z}"
        return jsonify(resp)





@app.route('/catch')
def catch():
    print("catch")
    ros2_node.publish_message()
    return render_template('index.html');




def gen():
    """Video streaming generator function."""
    yield b'--frame\r\n'
    while True:
        global frame
        if frame is not None:
            (flag, encodedImage) = cv2.imencode(".jpg", frame)  
            yield b'Content-Type: image/jpeg\r\n\r\n' + bytearray(encodedImage)  + b'\r\n--frame\r\n'
def gen2():
    """Video streaming generator function."""
    yield b'--frame\r\n'
    while True:
        global frame2
        if frame2 is not None:
            (flag, encodedImage) = cv2.imencode(".jpg", frame2)  
            yield b'Content-Type: image/jpeg\r\n\r\n' + bytearray(encodedImage)  + b'\r\n--frame\r\n'

@app.route('/video_feed')
def video_feed():
    """Video streaming route. Put this in the src attribute of an img tag."""
    return Response(gen(),
                    mimetype='multipart/x-mixed-replace; boundary=--frame')
@app.route('/video_feed2')
def video_feed2():
    """Video streaming route. Put this in the src attribute of an img tag."""
    return Response(gen2(),
                    mimetype='multipart/x-mixed-replace; boundary=--frame')

@app.route('/latest_message')
def get_current_time():
    return {'message': ros2_node.latest_message}

@app.route('/publish_message')
def get_publish_message():
    ros2_node.publish_message()
    return {}



# Simple state tracking
gripper_state = "closed"

@app.get("/api/gripper/open")
def open_gripper():
    global gripper_state
    return control_gripper_action('open')

@app.get("/api/gripper/close")
def close_gripper():
    global gripper_state
    return control_gripper_action('close')

@app.get("/api/gripper/toggle")
def toggle_gripper():
    global gripper_state
    action = 'close' if gripper_state == 'open' else 'open'
    return control_gripper_action(action)

def control_gripper_action(action):
    global gripper_state
    
    try:
        service_name = f'/{action}_gripper_service'
        
        print(f"Calling {service_name}...")
        result = subprocess.run(
            ['ros2', 'service', 'call', service_name, 'std_srvs/srv/Trigger', '{}'],
            capture_output=True,
            text=True,
            timeout=10  # Add timeout to prevent hanging
        )
        
        if result.returncode != 0:
            return jsonify({
                "success": False, 
                "error": result.stderr or "Service call failed"
            }), 500
        
        # Update state on success
        gripper_state = action
        
        return jsonify({
            "success": True,
            "state": gripper_state,
            "output": result.stdout
        })
        
    except subprocess.TimeoutExpired:
        return jsonify({"success": False, "error": "Service call timed out"}), 500
    except Exception as e:
        return jsonify({"success": False, "error": str(e)}), 500

@app.get("/api/gripper/state")
def get_state():
    return jsonify({"success": True, "state": gripper_state})

@app.get("/api/gripper/toggle_old")
def toggle_gripper_old():
    try:
        # Call ROS2 service
        result = subprocess.run(
            ['ros2', 'service', 'call', '/open_gripper_service', 'std_srvs/srv/Trigger', '{}'],
            capture_output=True,
            text=True
        )
        return jsonify({"success": True, "output": result.stdout})
    except Exception as e:
        return jsonify({"success": False, "error": str(e)})
    
LOG_FILE_PATH = os.path.join(os.getcwd(), 'logs', 'system.log')

LOG_FILE_PATH = "/home/jimmy/Downloads/mushroomproject/harvest_ws/camera.log"

# 用于去除终端颜色代码 (ROS 日志常带颜色)
ansi_escape = re.compile(r'\x1B(?:[@-Z\\-_]|\[[0-?]*[ -/]*[@-~])')

def parse_log_line(line, counter):
    """
    兼容多种日志格式的解析器
    """
    # 1. 去除首尾空白和 ANSI 颜色代码
    line = ansi_escape.sub('', line.strip())
    if not line:
        return None

    # --- 格式 A: Python/Loguru 格式 ---
    # 例子: 2026-01-28 14:53:56.187 | INFO | module:func:123 - message
    # 正则逻辑:
    # 1. (时间): 20xx-xx-xx xx:xx:xx.xxx
    # 2. 分隔符: |
    # 3. (级别): 单词
    # 4. 分隔符: |
    # 5. (模块): 任意字符直到 " - "
    # 6. (消息): 剩余内容
    pat_python = r"^(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}\.\d{3})\s+\|\s+(\w+)\s+\|\s+(.*?)\s+-\s+(.*)$"
    match_python = re.match(pat_python, line)

    if match_python:
        # 提取时间 (只取 HH:MM:SS.mmm)
        full_time = match_python.group(1)
        time_part = full_time.split(" ")[1] 
        
        return {
            "id": counter,
            "timestamp": time_part,
            "level": match_python.group(2).lower().strip(),
            # 模块名可能很长，可以只取第一部分，或者全部保留
            "module": match_python.group(3).split(":")[0], 
            "message": match_python.group(4)
        }

    # --- 格式 B: ROS 2 格式 ---
    # 例子: [INFO] [1769633636.239] [node_name]: message
    # 正则逻辑:
    # 1. [级别]
    # 2. [Unix时间戳]
    # 3. [节点名]:
    # 4. 消息
    pat_ros = r"^\[(\w+)\]\s+\[(\d+\.\d+)\]\s+\[(.*?)\]:\s+(.*)$"
    match_ros = re.match(pat_ros, line)

    if match_ros:
        # 处理 ROS 级别 (INFO -> info)
        level = match_ros.group(1).lower()
        
        # 处理 Unix 时间戳 -> 可读时间
        try:
            ts = float(match_ros.group(2))
            dt = datetime.datetime.fromtimestamp(ts)
            time_str = dt.strftime("%H:%M:%S.%f")[:-3] # HH:MM:SS.mmm
        except:
            time_str = match_ros.group(2) # 转换失败则显示原始数据

        return {
            "id": counter,
            "timestamp": time_str,
            "level": level,
            "module": match_ros.group(3), # 节点名
            "message": match_ros.group(4)
        }

    # --- 兜底: 无法识别的格式 ---
    return {
        "id": counter,
        "timestamp": datetime.datetime.now().strftime("%H:%M:%S"),
        "level": "info", # 默认为 info，避免前端 badge 报错
        "module": "RAW",
        "message": line
    }


def tail_file(filename):
    """
    生成器：模拟 tail -f 功能
    """
    # 确保文件存在，不存在则创建
    if not os.path.exists(filename):
        open(filename, 'a').close()

    with open(filename, 'r', encoding='utf-8') as f:
        # seek(0, 2) 将指针移动到文件末尾
        # 这样只读取连接后新产生的日志，不读取历史日志
        f.seek(0, 2)
        
        counter = 1
        
        while True:
            # 读取一行
            line = f.readline()
            
            if not line:
                # 如果没有新行，休息 0.1 秒避免 CPU 占用过高
                time.sleep(0.1)
                continue
            
            # 解析日志
            log_data = parse_log_line(line, counter)
            
            if log_data:
                counter += 1
                # 构造 SSE 格式数据
                json_data = json.dumps(log_data)
                yield f"data: {json_data}\n\n"



@app.route('/stream-logs')
def stream_logs():
    return Response(tail_file(LOG_FILE_PATH), mimetype="text/event-stream")



def generate():
    global latest_frame
    while True:
        with frame_lock:
            frame = latest_frame

        if frame is not None:
            buf = BytesIO()
            #frame.save(buf, format='JPEG')
            frame.convert("RGB").save(buf, format='JPEG', quality=80)

            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' +
                   buf.getvalue() +
                   b'\r\n')

        time.sleep(0.03)


@app.route('/robot_image')
def robot_image():
    return Response(generate(),
        mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/robot_image_old')
def get_robot_image():
    if not ros2_node.renderer:
        return "URDF not loaded", 500

    # 1. Get latest state from ROS thread
    current_joints = shared_state.get_snapshot()
    
    # 2. Render image using PyBullet
    pil_image = ros2_node.renderer.render(current_joints)
    
    # 3. Save to buffer and serve
    img_io = BytesIO()
    pil_image.save(img_io, 'PNG')
    img_io.seek(0)
    
    return send_file(img_io, mimetype='image/png')
