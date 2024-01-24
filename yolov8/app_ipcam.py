#!/usr/bin/env python
from importlib import import_module
import os

import redis
from flask import Flask, render_template, Response, jsonify

from threading import Event
import requests
from flask import Flask, render_template, request, jsonify
from flask_mqtt import Mqtt
# from flask_socketio import SocketIO
#import video_dir
# import car_dir
# import motor
import redis
import serial
import datetime
import time
import os
from time import sleep

import RPi.GPIO as GPIO
import time
import publish

# import camera driver
if os.environ.get('CAMERA'):
    Camera = import_module('camera_' + os.environ['CAMERA']).Camera
else:
    from camera_ipcam import Camera

# Raspberry Pi camera module (requires picamera package)
#from camera_pi import Camera


topic = "/flask/stop"
# Pin Definitions
input_pin = 12  # BCM pin 18, BOARD pin 12

prev_value = None
# GPIO.setmode(GPIO.BOARD)  # BCM pin-numbering scheme from Raspberry Pi
# GPIO.setup(input_pin, GPIO.IN)  # set pin as an input pin


# import config

busnum = 1  # Edit busnum to 0, if you uses Raspberry Pi 1 or 0

# video_dir.setup(busnum=busnum)
# motor.setup(busnum=busnum)     # Initialize the Raspberry Pi GPIO connected to the DC motor.
# motor.setSpeed(40)
# video_dir.home_x_y()

i = 0

#ser = serial.Serial("/dev/ttyACM0", 115200)

redis_server = ''
broker = ''
try:
    for line in open("../ip.txt"):
        if line[0:6] == "broker":
            broker = line[9:len(line) - 1]
        if line[0:6] == "reddis":
            redis_server = line[9:len(line) - 1]
except:
    pass
#broker = broker.replace("\n", "").replace("\r\n", "")
#broker = "192."
print(broker)
print(redis_server)
pool = redis.ConnectionPool(host=redis_server, port=6379, decode_responses=True, password='jimmy')
r = redis.Redis(connection_pool=pool)

app = Flask(__name__)
app.config['DEBUG'] = False
#broker = '192.168.1.4'
app.config['MQTT_BROKER_URL'] = broker
# app.config['MQTT_BROKER_URL'] = '10.0.0.134'
# app.config['MQTT_BROKER_URL'] = '192.168.254.42'
app.config['MQTT_BROKER_PORT'] = 1883
app.config['MQTT_USERNAME'] = ''  # Set this item when you need to verify username and password
app.config['MQTT_PASSWORD'] = ''  # Set this item when you need to verify username and password
app.config['MQTT_KEEPALIVE'] = 5  # Set KeepAlive time in seconds
app.config['MQTT_TLS_ENABLED'] = False  # If your server supports TLS, set it True
# socketio = SocketIO(app)
topic = '/flask/scan'
topic2 = '/flask/xyz'
topic3 = '/flask/serial'
topic4 = '/flask/pickup'
topic5 = '/flask/home'
topic6 = '/flask/drop'
topic7 = '/flask/stop'
mqtt_client = Mqtt(app)
y = 0


# app.config.from_object(config)


def send_wake_up(ser):
    # Wake up
    # Hit enter a few times to wake the Printrbot
    ser.write(str.encode("\r\n\r\n"))
    time.sleep(2)  # Wait for Printrbot to initialize
    ser.flushInput()  # Flush startup text in serial input


def wait_for_movement_completion(ser, cleaned_line):
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
    # send_wake_up(ser)
    if command:  # checks if string is empty
        print("Sending gcode:" + str(command))
        # converts string to byte encoded string and append newline
        command = str.encode(command)
        # command = str.encode(command + x'\r\n')
        ser.write(command)  # Send g-code
        # wait_for_movement_completion(ser, command)
        # grbl_out = ser.readline()  # Wait for response with carriage return
        # print(" : ", grbl_out.strip().decode('utf-8'))
        return 'ok'
        # return grbl_out.strip().decode('utf-8')


@app.route('/scan')
def scan():
    global prev_value
    # video_dir.move_decrease_y(10)
    r.set("mode", "camera_ready")
    #r.set("global_camera_xy","0,0")
    r.delete("detections")
    #loop_pickup();
    return render_template('index.html');


@app.route('/xbackward')
def xbackward():
    command(ser, "$X\r\n")
    time.sleep(1)
    command(ser, "G21 G91 G1 X1 F500\r\n")
    xy = r.get("global_camera_xy").split(",")
    print(xy)
    r.set("global_camera_xy", str(int(xy[0]) - 25) + "," + xy[1])
    return render_template('index.html');


@app.route('/xforward')
def xfarward():
    command(ser, "$X\r\n")
    time.sleep(1)
    command(ser, "G21 G91 G1 X-1 F500\r\n")
    xy = r.get("global_camera_xy").split(",")
    print(xy)
    r.set("global_camera_xy", str(int(xy[0]) + 25) + "," + xy[1])
    return render_template('index.html');




@app.route('/catch')
def catch():
    video_dir.move_increase_y(10)
    print("catch")
    return render_template('index.html');


@app.route('/release')
def release():
    video_dir.move_decrease_y(10)
    return render_template('index.html');
@app.route('/zup')
def zup():
    video_dir.move_decrease_x(10)
    return render_template('index.html');
@app.route('/zdown')
def zdown():

    video_dir.move_increase_x(10)
    return render_template('index.html');


@app.route('/yforward')
def yforward():
    command(ser, "$X\r\n")
    time.sleep(1)
    # Moving forward code
    command(ser, "G21 G91 G1  Y-5 F500\r\n")
    xy = r.get("global_camera_xy").split(",")
    print(xy)
    r.set("global_camera_xy", xy[0] + "," + str(int(xy[1]) + 60))
    return render_template('index.html');


@app.route('/ybackward')
def ybackward():
    command(ser, "$X\r\n")
    time.sleep(1)
    # Moving forward code
    command(ser, "G21 G91 G1  Y5 F500\r\n")
    xy = r.get("global_camera_xy").split(",")
    print(xy)
    r.set("global_camera_xy", xy[0] + "," + str(int(xy[1]) + 60))
    return render_template('index.html');



def autopick():
    publish_result = mqtt_client.publish(topic, "flask/scan")
    command(ser, "G21 G91 G1 X1 F2540\r\n")
    return render_template('index.html');


@app.route('/zerosetting')
def zero():
    # publish_result = mqtt_client.publish(topic, "/flask/home")
    command(ser, "$X\r\n")
    time.sleep(1)
    command(ser, "$H\r\n")
    r.set("global_camera_xy", "0,0")
    return render_template('index.html');


@app.route('/home')
def home():
    # publish_result = mqtt_client.publish(topic, "/flask/home")
    command(ser, "$X\r\n")
    time.sleep(1)
    command(ser, "G28 G91 X0.0 Y0.00 F500\r\n")
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
                    mimetype='multipart/x-mixed-replace; boundary=frame')


if __name__ == '__main__':
    app.run(host='0.0.0.0', threaded=True,port='5001')
