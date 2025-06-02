import io
from loguru import logger
import rclpy
import signal
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
import threading
import time
import board
import busio




import redis
from flask import Flask, render_template, Response, jsonify
from threading import Event
from flask import Flask, render_template, request, jsonify
from flask_mqtt import Mqtt

#from cv_bridge import CvBridge,CvBridgeError
#import cv2
frame=None

import time


from camera_usb import Camera
import cv2


#from ultralytics import YOLO
import numpy as np
from cv_bridge import CvBridge,CvBridgeError



# Configure min and max servo pulse lengths
servo_min = 250  # Min pulse length out of 4096
#servo_tmp=servo_min
servo_inc=50
servo_max = 400  # Max pulse length out of 4096
frame=None
boxing_img=None
class MovePublisher(Node):
    def __init__(self):
        super().__init__('test_publisher')
        self.pub_rpi5_raw_img = self.create_publisher(Image,"/yolox/rpi5/raw_image", 10)
        self.sub_boxing_img = self.create_subscription(Image,"/yolox/rpi5/boxing_image",self.imageflow_callback, 10)
        #self._adjust_publisher = self.create_publisher(String, '/yolox/move/adjust/xy', 1)
        #self.subscription = self.create_subscription(Image,'/yolox/boxes_image',self.chatter_callback,10)
        #self.gripper_open_subs= self.create_subscription(String,'/yolox/gripper_open',self.gripper_open_callback,10)
        #self.gripper_hold_subs = self.create_subscription(String,'/yolox/gripper_hold',self.gripper_hold_callback,10)
        #self.gripper_hold_subs = self.create_subscription(String,'/yolox/move/detected',self.gripper_detected_move_callback,10)

        self.latest_message = None
        self.bridge = CvBridge()
    
    def imageflow_callback(self,msg:Image) -> None:
            global frame,boxing_img
            boxing_img = self.bridge.imgmsg_to_cv2(msg,"bgr8")
            boxing_img=cv2.imencode('.jpg', boxing_img)[1].tobytes()
            frame=None
            boxing_img=None


                

    def chatter_callback(self, msg):
        global frame
        #print(f'chatter cb received: {msg.data}')
        #frame = self.bridge.imgmsg_to_cv2(msg,"bgr8")
        #self.latest_message = msg.data
        #frame = msg.data
   




    def publish_message(self,msg):
        self.publisher.publish(msg)
    def gen(self,camera):
        global frame,boxing_img
        """Video streaming generator function."""
        yield b'--frame\r\n'
        while True:
            frame = camera.get_frame()
            #nparr = np.fromstring(frame, np.uint8)
            #ogsimg=cv2.imdecode(nparr, cv2.IMREAD_COLOR)

            #logger.info(frame.type)
            jpg_as_np = np.frombuffer(frame, dtype=np.uint8)
            img = cv2.imdecode(jpg_as_np, flags=1)
            img_pub = self.bridge.cv2_to_imgmsg(img,"bgr8")
            self.pub_rpi5_raw_img.publish(img_pub)
            logger.info("box image:{}".format(boxing_img))
            
            if frame==None:
                yield b'Content-Type: image/jpeg\r\n\r\n' + boxing_img + b'\r\n--frame\r\n'
            else:
                yield b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n--frame\r\n'
            #frame=None



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
threading.Thread(target=ros2_thread, args=[ros2_node]).start()
prev_sigint_handler = signal.signal(signal.SIGINT, sigint_handler)

@app.route('/')
def index():
    return render_template('index.html')


@app.route('/msg')
def msg():
    ros2_node.publish_message()
    return render_template('index.html')

@app.route('/xbackward')
def xbackward():
    ros2_node.publish_message()
    return render_template('index.html');


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



@app.route('/xforward')
def xfarward():
    msg = String()
    msg.data = 'x:' % self.i
    ros2_node.publish_message()
    ros2_node.publish_message()
    return render_template('index.html');

@app.route('/catch')
def catch():
    print("catch")
    ros2_node.publish_message()
    return render_template('index.html');


@app.route('/release')
def release():
    ros2_node.publish_message()
    return render_template('index.html');
@app.route('/zup')
def zup():
    ros2_node.publish_message()
    return render_template('index.html');
@app.route('/zdown')
def zdown():
    ros2_node.publish_message()
    return render_template('index.html');







@app.route('/zerosetting')
def zero():
    return render_template('index.html');


@app.route('/home')
def home():
    return render_template('index.html');



@app.route('/video_feed')
def video_feed():
    return Response(ros2_node.gen(Camera()),mimetype='multipart/x-mixed-replace; boundary=frame')


