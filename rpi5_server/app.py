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


from camera_usb import Camera
#from camera_pi import Camera
import cv2
#from picamera2 import Picamera2

from ultralytics import YOLO
import numpy as np
from cv_bridge import CvBridge,CvBridgeError
# Set up the camera with Picam
#picam2 = Picamera2()
#picam2.preview_configuration.main.size = (1280, 1280)
#picam2.preview_configuration.main.format = "RGB888"
#picam2.preview_configuration.align()
#picam2.configure("preview")
#picam2.start()


redis_server='172.27.34.62'
pool = redis.ConnectionPool(host=redis_server, port=6379, decode_responses=True,password='jimmy')
r = redis.Redis(connection_pool=pool)

# Load YOLOv11
model = YOLO("/home/pi/yolomodel/yolo11n_mushroom_ncnn_model")
model2 = YOLO("/home/pi/yolomodel/shape_yolo11_ncnn_model")






# Configure min and max servo pulse lengths
servo_min = 300  # Min pulse length out of 4096
servo_inc=50
servo_max = 550  # Max pulse length out of 4096
frame=None
boxing_img=None
class MovePublisher(Node):
    def __init__(self):
        super().__init__('test_publisher')
        self.pub_rpi5_raw_img = self.create_publisher(Image,"/yolox/rpi5/raw_image", 10)
        self.sub_boxing_img = self.create_subscription(Image,"/yolox/rpi5/boxing_image",self.imageflow_callback, 10)
        self.gripper_adjust_pub= self.create_publisher(String, '/yolox/move/adjust/xy', 1)
        #self.subscription = self.create_subscription(Image,'/yolox/boxes_image',self.chatter_callback,10)
        self.gripper_open_subs= self.create_subscription(String,'/yolox/gripper_open',self.gripper_open_callback,10)
        self.gripper_hold_subs = self.create_subscription(String,'/yolox/gripper_hold',self.gripper_hold_callback,10)
        #self.gripper_hold_subs = self.create_subscription(String,'/yolox/move/detected',self.gripper_detected_move_callback,10)

        self.latest_message = None
        self.bridge = CvBridge()
    
    def imageflow_callback(self,msg:Image) -> None:
            global frame,boxing_img
            boxing_img = self.bridge.imgmsg_to_cv2(msg,"bgr8")
            frame=cv2.imencode('.jpg', boxing_img)[1].tobytes()
            boxing_img=None


                

    def chatter_callback(self, msg):
        global frame
        #print(f'chatter cb received: {msg.data}')
        #frame = self.bridge.imgmsg_to_cv2(msg,"bgr8")
        #self.latest_message = msg.data
        #frame = msg.data
    def gripper_hold_callback(self, msg):
	#    print(f'hold cb received: {msg.data}')
        pwm.set_pwm(0, 0, 480)
        pwm.set_pwm(1, 0, 480)
        pwm.set_pwm(2, 0, 480)
        time.sleep(1)
        #print("servo_tmp={},{:>5}\t{:>5.3f}".format(servo_tmp,chan.value, chan.voltage))

    def gripper_detected_move_callback(self, msg):
    
        if result.boxes != None:
            adjust_msg = String()
            adjust_msg.data = 'yes,%d,%d,%d' %(int(5),int(5),0) 
            self._adjust_publisher.publish(adjust_msg)

            # Get inference time
            #inference_time = results[0].speed['inference']
            #fps = 1000 / inference_time  # Convert to milliseconds
            #text = f'FPS: {fps:.1f}'

            # Define font and position
            #font = cv2.FONT_HERSHEY_SIMPLEX
            #text_size = cv2.getTextSize(text, font, 1, 2)[0]
            #text_x = annotated_frame.shape[1] - text_size[0] - 10  # 10 pixels from the right
            #text_y = text_size[1] + 10  # 10 pixels from the top

            # Draw the text on the annotated frame
            #cv2.putText(annotated_frame, text, (text_x, text_y), font, 1, (255, 255, 255), 2, cv2.LINE_AA)
        else:
            adjust_msg = String()
            adjust_msg.data = 'no,%d,%d,%d' %(int(5),int(5),hi.z) 
            self._adjust_publisher.publish(adjust_msg)


    def gripper_open_callback(self, msg):
        global servo_min
        print(f'open cb received: {msg.data}')
        pwm.set_pwm(0, 0, servo_min)
        pwm.set_pwm(1, 0, servo_min)
        pwm.set_pwm(2, 0, servo_min)
        #pwm.set_pwm(4, 0, servo_min)
        time.sleep(1)



    def publish_message(self,msg):
        self.publisher.publish(msg)
    def gen(self,camera):
        global frame,boxing_img
        """Video streaming generator function."""
        yield b'--frame\r\n'
        while True:
            frame = camera.get_frame()
            #adjust_gripper_center=r.get("adjust_gripper_center")
            #logger.info(adjust_gripper_center)
            #if(adjust_gripper_center!=None):
            ##    gripper_msg2 = String()
            #    gripper_msg2.data = adjust_gripper_center
            #    self.gripper_adjust_pub.publish(gripper_msg2)
            #nparr = np.fromstring(frame, np.uint8)
            #ogsimg=cv2.imdecode(nparr, cv2.IMREAD_COLOR)

            #logger.info(frame.type)
            jpg_as_np = np.frombuffer(frame, dtype=np.uint8)
            img = cv2.imdecode(jpg_as_np, flags=1)
            detect_res=model(img,conf=0.8)
            logger.info(detect_res!=None)
            if detect_res!=None and( r.get("mode")=="adjust_camera_init" or r.get("mode")=="adjust_camera_done" ):
                #detect_res=model(img,conf=0.8)    
                #frame=detect_res[0].plot()
                
                boxes = detect_res[0].boxes.cpu().numpy()
                xyxy = boxes.xyxy
                classes = boxes.cls
                confs = boxes.conf
                    #print(detect_res)
                line_color=(255, 0, 255)
                label_color=(255, 255, 255)
                line_thickness=2
                for (x1, y1, x2, y2), conf, class_id in zip(xyxy,confs,classes):
                    print(x1,y1,x2,y2)
                    left, top, right, bottom = int(x1), int(y1), int(x2), int(y2)
                    adjust_gripper_center=str(int((right + left-640) / 2))+","+str(int((top + bottom-640) / 2))
                    logger.info("adjust_gripper_center:{},{}".format(int((right + left-640) / 2),int((top + bottom-640) / 2)))
                    if(abs(int((right + left-640) / 2))>50 or abs(int((top + bottom-640) / 2))>50 ):
                        r.set("adjust_gripper_center",str(int((right + left-640) / 2))+","+str(int((top + bottom-640) / 2)))
                    else:
                         r.set("adjust_gripper_center","")

                    gripper_msg2 = String()
                    gripper_msg2.data = adjust_gripper_center
                    self.gripper_adjust_pub.publish(gripper_msg2)

                    width = right - left
                    height = bottom - top
                    center = (left + int((right - left) / 2), top + int((bottom - top) / 2))
                    label = "mushroom"
                    confidence = conf
                    label= label+",center:"+str(int((right + left) / 2))+","+str(int((top + bottom) / 2))
                        #if(label=="laptop"):
                    cv2.rectangle(img, (left, top), (right, bottom), (255, 0, 0), 2)
                    cv2.putText(img, label, center, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 1, cv2.LINE_AA)
                    break
            else:
                r.set("adjust_gripper_center","")
                
            frame=cv2.imencode('.jpg', img)[1].tobytes()
            yield b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n--frame\r\n'
            frame=None



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


