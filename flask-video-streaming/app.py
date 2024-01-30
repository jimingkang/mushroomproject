#!/usr/bin/env python
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

import RPi.GPIO as GPIO
import time
import publish


topic = '/flask/scan'
topic2 = '/flask/xyz'
topic3 = '/flask/serial'
topic4 = '/flask/pickup'
topic5 = '/flask/home'
topic6 = '/flask/drop'
topic7 = '/flask/stop'
#ser = serial.Serial("/dev/ttyACM0",115200)
#video_dir.setup(busnum=1)
#motor.setup(busnum=busnum)     # Initialize the Raspberry Pi GPIO connected to the DC motor. 
#motor.setSpeed(40)
#video_dir.home_x_y()
# import camera driver
if os.environ.get('CAMERA'):
    Camera = import_module('camera_' + os.environ['CAMERA']).Camera
else:
    from camera_rs import Camera
# Raspberry Pi camera module (requires picamera package)

app = Flask(__name__)

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


app.config['MQTT_BROKER_URL'] = broker
app.config['MQTT_BROKER_PORT'] = 1883
app.config['MQTT_USERNAME'] = ''  # Set this item when you need to verify username and password
app.config['MQTT_PASSWORD'] = ''  # Set this item when you need to verify username and password
app.config['MQTT_KEEPALIVE'] = 5  # Set KeepAlive time in seconds
app.config['MQTT_TLS_ENABLED'] = False  # If your server supports TLS, set it True
mqtt_client = Mqtt(app)



@app.route('/scan')
def scan():
    return render_template('index.html');


@app.route('/xbackward')
def xbackward():
    return render_template('index.html');


@app.route('/xforward')
def xfarward():
 
    return render_template('index.html');




@app.route('/catch')
def catch():
    return render_template('index.html');


@app.route('/release')
def release():
    video_dir.move_decrease_y(10)
    return render_template('index.html');
@app.route('/zup')
def zup():
    command(ser, "G21 G91 G1  Z1 F500\r\n")

    return render_template('index.html');

@app.route('/zdown')
def zdown():

    command(ser, "G21 G91 G1  Z-1 F500\r\n")
    return render_template('index.html');


@app.route('/yforward')
def yforward():
    command(ser, "$X\r\n")
    time.sleep(1)
    # Moving forward code
    command(ser, "G21 G91 G1  Y-1 F500\r\n")
    xy = r.get("global_camera_xy").split(",")
    print(xy)
    r.set("global_camera_xy", xy[0] + "," + str(int(xy[1]) + 60))
    return render_template('index.html');


@app.route('/ybackward')
def ybackward():
    return render_template('index.html');



def autopick():
    publish_result = mqtt_client.publish(topic, "flask/scan")
    command(ser, "G21 G91 G1 X1 F2540\r\n")
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
    app.run(host='0.0.0.0', threaded=True)
    #app.run(host='192.168.1.3', threaded=True)
