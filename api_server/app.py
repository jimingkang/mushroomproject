import rclpy
import signal
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
import threading

import redis
from flask import Flask, render_template, Response, jsonify
from threading import Event
from flask import Flask, render_template, request, jsonify
from flask_mqtt import Mqtt

from cv_bridge import CvBridge,CvBridgeError
import cv2
frame=None
class MovePublisher(Node):
    def __init__(self):
        super().__init__('test_publisher')
        self.publisher = self.create_publisher(String, '/move/x', 1)
        self.subscription = self.create_subscription(Image,'/yolox/boxes_image',self.chatter_callback,10)
        self.latest_message = None
        self.bridge = CvBridge()

    def chatter_callback(self, msg):
        global frame
        #print(f'chatter cb received: {msg.data}')
        frame = self.bridge.imgmsg_to_cv2(msg,"bgr8")
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





def gen():
    """Video streaming generator function."""
    yield b'--frame\r\n'
    while True:
        global frame
        (flag, encodedImage) = cv2.imencode(".jpg", frame)
        yield b'Content-Type: image/jpeg\r\n\r\n' + bytearray(encodedImage)  + b'\r\n--frame\r\n'


@app.route('/video_feed')
def video_feed():
    """Video streaming route. Put this in the src attribute of an img tag."""
    return Response(gen(),
                    mimetype='multipart/x-mixed-replace; boundary=--frame')

@app.route('/latest_message')
def get_current_time():
    return {'message': ros2_node.latest_message}

@app.route('/publish_message')
def get_publish_message():
    ros2_node.publish_message()
    return {}
