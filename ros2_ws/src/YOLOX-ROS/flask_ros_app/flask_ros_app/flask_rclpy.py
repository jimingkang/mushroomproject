from importlib import import_module
import os
from flask import Flask, render_template, Response, jsonify
from threading import Event
import requests
from flask_mqtt import Mqtt
#import video_dir
#import car_dir
#import motor
import redis
import serial
import datetime
import time
import os
from time import sleep
import time


#from HitbotInterface import HitbotInterface
from flask_cors import CORS, cross_origin

topic = '/flask/scan'
topic2 = '/flask/xyz'
topic3 = '/flask/serial'
topic4 = '/flask/pickup'
topic5 = '/flask/home'
topic6 = '/flask/drop'
topic7 = '/flask/stop'





import rclpy
import signal
from rclpy.node import Node
from std_msgs.msg import String
import threading
from flask import Flask
from sensor_msgs.msg import Image
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library



class TestPublisher(Node):
    def __init__(self):
        super().__init__('test_publisher')
        self.publisher = self.create_publisher(String, 'flask_pub_topic', 10)
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/depth/image_rect_raw',
            self.sub_callback,
            10)
        self.latest_message = None
        self.br = CvBridge()
        self.current_frame=None
    def sub_callback(self, msg):
        #print(f'chatter cb received: {msg.data}')
        #self.latest_message = msg.data
    	# Display the message on the console
        #self.get_logger().info('Receiving video frame')
    	# Convert ROS Image message to OpenCV image
        self.current_frame = self.br.imgmsg_to_cv2(msg)
        # Display image
        cv2.imwrite("camera", self.current_frame)

    def publish_message(self):
        msg = String()
        msg.data = 'hello, world!'
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
pool = redis.ConnectionPool(host=redis_server, port=6379, decode_responses=True,password='jimmy')
r = redis.Redis(connection_pool=pool)



rclpy.init(args=None)
ros2_node = TestPublisher()
app = Flask(__name__)
app.config['MQTT_BROKER_URL'] = broker
app.config['MQTT_BROKER_PORT'] = 1883
app.config['MQTT_USERNAME'] = ''  # Set this item when you need to verify username and password
app.config['MQTT_PASSWORD'] = ''  # Set this item when you need to verify username and password
app.config['MQTT_KEEPALIVE'] = 5  # Set KeepAlive time in seconds
app.config['MQTT_TLS_ENABLED'] = False  # If your server supports TLS, set it True
cors = CORS(app)
app.config['CORS_HEADERS'] = 'Content-Type'
mqtt_client = Mqtt(app)

threading.Thread(target=ros2_thread, args=[ros2_node]).start()
prev_sigint_handler = signal.signal(signal.SIGINT, sigint_handler)


@app.route('/latest_message')
def get_current_time():
    return {'message': ros2_node.latest_message}

@app.route('/publish_message')
def get_publish_message():
    ros2_node.publish_message()
    return {}



@app.route('/scan')
def scan():
    return render_template('index.html');


@app.route('/xbackward')
def xbackward():
    hi.get_scara_param()
    hi.new_movej_xyz_lr(hi.x-10,hi.y,hi.z,0,70,0,1)
    hi.get_scara_param()
    r.set("global_camera_xy",hi.x+","+hi.y)
    return ;


@app.route('/xforward')
def xfarward():
    hi.get_scara_param()
    hi.new_movej_xyz_lr(hi.x+10,hi.y,hi.z,0,70,0,1)
    hi.get_scara_param()
    r.set("global_camera_xy",hi.x+","+hi.y)
    return ;

@app.route('/catch')
def catch():
    return render_template('index.html');


@app.route('/release')
def release():
    return render_template('index.html');
@app.route('/zup')
def zup():
    return render_template('index.html');

@app.route('/zdown')
def zdown():
    return render_template('index.html');


@app.route('/yforward')
def yforward():
    return render_template('index.html');


@app.route('/ybackward')
def ybackward():
    return render_template('index.html');





def autopick():
    publish_result = mqtt_client.publish(topic, "flask/scan")
    return render_template('index.html');


@app.route('/zerosetting')
def zero():
    return render_template('index.html');


@app.route('/home')
def home():
    return render_template('index.html');





@app.route('/')
def index():
    return render_template('index.html')



@app.route('/update_mushroom_map' ,methods=['GET'])
def update_mushroom_map():
    detections=r.hgetall("detections_history")
    print(jsonify({"response":detections}))
    return jsonify({"response":detections})


def gen():
    global ros2_node
    """Video streaming generator function."""
    yield b'--frame\r\n'
    while True:
        frame = ros2_node.current_frame
        print(frame==None)
        if frame != None:
            yield b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n--frame\r\n'


@app.route('/video_feed')
def video_feed():
    """Video streaming route. Put this in the src attribute of an img tag."""
    return Response(gen(),mimetype='multipart/x-mixed-replace; boundary=frame')



