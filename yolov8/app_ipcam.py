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
import math

import RPi.GPIO as GPIO
import time
import publish
from HitbotInterface import HitbotInterface

from paho.mqtt import client as mqtt_client

#hi=HitbotInterface(92); #//92 is robotid
#hi.net_port_initial()
#ret=hi.initial(1,210); #
#print(hi.is_connect())
#print(hi.unlock_position())
#hi.movel_xyz(0,0,0,25,20)

# import camera driver
if os.environ.get('CAMERA'):
    Camera = import_module('camera_' + os.environ['CAMERA']).Camera
else:
    from camera_ipcam import Camera

# Raspberry Pi camera module (requires picamera package)
#from camera_pi import Camera





# Pin Definitions
input_pin = 12  # BCM pin 18, BOARD pin 12

prev_value = None


i = 0



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
print(broker)
print(redis_server)
pool = redis.ConnectionPool(host=redis_server, port=6379, decode_responses=True, password='jimmy')
r = redis.Redis(connection_pool=pool)

app = Flask(__name__)
app.config['DEBUG'] = False
app.config['MQTT_BROKER_URL'] = broker
app.config['MQTT_BROKER_PORT'] = 1883
app.config['MQTT_USERNAME'] = ''  # Set this item when you need to verify username and password
app.config['MQTT_PASSWORD'] = ''  # Set this item when you need to verify username and password
app.config['MQTT_KEEPALIVE'] = 5  # Set KeepAlive time in seconds
app.config['MQTT_TLS_ENABLED'] = False  # If your server supports TLS, set it True

topic = '/flask/scan'
topic2 = '/flask/xyz'
topic3 = '/flask/serial'
topic4 = '/flask/pickup'
topic5 = '/flask/home'
topic6 = '/flask/drop'
topic7 = '/flask/stop'
mqtt_client = Mqtt(app)
y = 0



@app.route('/move/<string:direction>')
def move(direction,amount=0,custom_x=0,custom_y=0):
    global camera_x,camera_y,_movement_step,_feedback,hi
    # Open the serial port
    #ser = serial.Serial("/dev/ttyACM0", 115200, timeout=5)
    if direction == "home":
        hi.movej_angle(0,0,0,0, 100, 0)
        hi.wait_stop()
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
        return f"{rett}<br>x = {x} {type(x)}<br>y = {y} {type(y)}<br>z = {z} {type(z)}<br>roughly = {roughly} {type(roughly)}"
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
        
        return f"{direction}\nx = {x}\ny = {y}\nz = {z}"



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



@mqtt_client.on_connect()
def handle_connect(client, userdata, flags, rc):
    if rc == 0:
        print('App_ipcam topic3 =/flask/serial Connected successfully')
        mqtt_client.subscribe(topic3)  # subscribe topic
    else:
        print('Bad connection. Code:', rc)


@mqtt_client.on_message()
def handle_mqtt_message(client, userdata, message):
    # print(message.topic)
    data = dict(topic=message.topic, payload=message.payload.decode())
    print('App_ipcam  Received message on topic: {topic} with payload: {payload}'.format(**data))
    xyz = data['payload']
    if message.topic == topic3:
        print("get xy payload=" + xyz)
        xyz = data['payload']
        print(xyz)
        real_xyz = xyz.split(",")

        # detections=r.hgetall("detections")
        # for item in r.hkeys("detections"):
        #    real_xyz=r.hget("detections",item).split(",")
        # real_xyz=detections[0].split(",")
        real_x = int(float(real_xyz[0]) * 1000)
        real_y = int(float(real_xyz[1]) * 1000)
        real_z = int(float(real_xyz[2]) * 1000)
        #:wqreal_y = real_y - 190
        #real_x = real_x + 10
        x =   real_x
        y = real_y
        z=-real_z
        if 1:#(abs(x) > 10 or abs(y) > 10):
            track_id=real_xyz[2]
            camera_xyz=r.get("global_camera_xy").split(",")
            cam_x=float(camera_xyz[0])
            cam_y=float(camera_xyz[1])
            print(cam_x)
            new_camera_x=x+cam_x
            new_camera_y=y+cam_y
            distance = int(math.sqrt(new_camera_x * new_camera_x + new_camera_y * new_camera_y))
            print(distance)
            detected_index=r.zrangebyscore("detections_index",min=distance-10,max=distance +10)
            print(len(detected_index))
            if len(detected_index)<1:
                obj=str(new_camera_x) + "," + str(new_camera_y) +"," + str(track_id)
                r.zadd("detections_index",{obj:distance} )
                r.hset("detections", str(distance), str(new_camera_x) + "," + str(new_camera_y) +"," + str(track_id))
                print("before move realx,y")
                print(x)
                print(y)
                #rett=hi.movel_xyz(hi.x+x,hi.y+y,hi.z,25,20)
                #hi.wait_stop()
                #print(rett)




if __name__ == '__main__':
    app.run(host='0.0.0.0', threaded=True,port='5001')
