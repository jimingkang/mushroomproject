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
import adafruit_ads1x15.ads1015 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

# Create the I2C bus
i2c = busio.I2C(board.SCL, board.SDA)
# Create the ADC object using the I2C bus
#ads = ADS.ADS1015(i2c)
# Create single-ended input on channel 0
#chan = AnalogIn(ads, ADS.P0)

import redis
from flask import Flask, render_template, Response, jsonify
from threading import Event
from flask import Flask, render_template, request, jsonify
from flask_mqtt import Mqtt

#from cv_bridge import CvBridge,CvBridgeError
#import cv2
frame=None

import time
import Adafruit_PCA9685
pwm = Adafruit_PCA9685.PCA9685(address=0x40, busnum=1)


import cv2
from picamera2 import Picamera2
from ultralytics import YOLO

# Set up the camera with Picam
picam2 = Picamera2()
picam2.preview_configuration.main.size = (1280, 1280)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()

# Load YOLOv8
model = YOLO("./yolo11n-seg_ncnn_model")
#model = YOLO("yolov8n.pt")





# Configure min and max servo pulse lengths
servo_min = 250  # Min pulse length out of 4096
#servo_tmp=servo_min
servo_inc=50
servo_max = 400  # Max pulse length out of 4096

class MovePublisher(Node):
    def __init__(self):
        super().__init__('test_publisher')
        self._adjust_publisher = self.create_publisher(String, '/yolox/move/adjust/xy', 1)
        #self.subscription = self.create_subscription(Image,'/yolox/boxes_image',self.chatter_callback,10)
        self.gripper_open_subs= self.create_subscription(String,'/yolox/gripper_open',self.gripper_open_callback,10)
        self.gripper_hold_subs = self.create_subscription(String,'/yolox/gripper_hold',self.gripper_hold_callback,10)
        self.gripper_hold_subs = self.create_subscription(String,'/yolox//move/detected',self.gripper_detected_move_callback,10)

        self.latest_message = None
        #self.bridge = CvBridge()

    def chatter_callback(self, msg):
        global frame
        #print(f'chatter cb received: {msg.data}')
        #frame = self.bridge.imgmsg_to_cv2(msg,"bgr8")
        #self.latest_message = msg.data
        #frame = msg.data
    def gripper_hold_callback(self, msg):
        servo_tmp=servo_min
        print(f' hold cb received: {msg.data}') 
        #while  servo_tmp<servo_max or chan.voltage<0.4:
        #    servo_tmp=servo_tmp+servo_inc
        pwm.set_pwm(0, 0, servo_tmp)
        pwm.set_pwm(1, 0, servo_tmp)
        pwm.set_pwm(2, 0, servo_tmp)
        pwm.set_pwm(4, 0, servo_tmp)
        time.sleep(0.05)
        #print("servo_tmp={},{:>5}\t{:>5.3f}".format(servo_tmp,chan.value, chan.voltage))

    def gripper_detected_move_callback(self, msg):
        frame = picam2.capture_array()
        # Run YOLO model on the captured frame and store the results
        results = model(frame)
        logger.info(results)
        
        # Output the visual detection data, we will draw this on our camera preview window
        annotated_frame = results[0].plot()
        if annotated_frame != None:
            adjust_msg = String()
            adjust_msg.data = 'yes,%d,%d,%d' %(int(5),int(5),hi.z) 
            self._adjust_publisher.publish(adjust_msg)

            # Get inference time
            inference_time = results[0].speed['inference']
            fps = 1000 / inference_time  # Convert to milliseconds
            text = f'FPS: {fps:.1f}'

            # Define font and position
            font = cv2.FONT_HERSHEY_SIMPLEX
            text_size = cv2.getTextSize(text, font, 1, 2)[0]
            text_x = annotated_frame.shape[1] - text_size[0] - 10  # 10 pixels from the right
            text_y = text_size[1] + 10  # 10 pixels from the top

            # Draw the text on the annotated frame
            cv2.putText(annotated_frame, text, (text_x, text_y), font, 1, (255, 255, 255), 2, cv2.LINE_AA)
        else:
            adjust_msg = String()
            adjust_msg.data = 'no,%d,%d,%d' %(int(5),int(5),hi.z) 
            self._adjust_publisher.publish(adjust_msg)


    def gripper_open_callback(self, msg):
        print(f'open cb received: {msg.data}')
        pwm.set_pwm(0, 0, servo_min)
        pwm.set_pwm(1, 0, servo_min)
        pwm.set_pwm(2, 0, servo_min)
        pwm.set_pwm(4, 0, servo_min)
        time.sleep(1)



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




@app.route('/latest_message')
def get_current_time():
    return {'message': ros2_node.latest_message}

@app.route('/publish_message')
def get_publish_message():
    ros2_node.publish_message()
    return {}
