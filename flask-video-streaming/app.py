#!/usr/bin/env python
from importlib import import_module
import os
from flask import Flask, render_template, Response, jsonify
from threading import Event
import requests
from flask_mqtt import Mqtt
import video_dir
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


ser = serial.Serial("/dev/ttyACM0",115200)
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
    #send_wake_up(ser)
    if command:  # checks if string is empty
        print("Sending gcode:" + str(command))
        # converts string to byte encoded string and append newline
        command = str.encode(command)
        #command = str.encode(command + x'\r\n')
        ser.write(command)  # Send g-code
        #wait_for_movement_completion(ser, command)
        #grbl_out = ser.readline()  # Wait for response with carriage return
        #print(" : ", grbl_out.strip().decode('utf-8'))
    return 'ok'
    #return grbl_out.strip().decode('utf-8')


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

@mqtt_client.on_connect()
def handle_connect(client, userdata, flags, rc):
   if rc == 0:
        print('Connected successfully')
        mqtt_client.subscribe(topic3) # subscribe topic
        #mqtt_client.subscribe(topic5) # subscribe topic
   else:
       print('Bad connection. Code:', rc)

@mqtt_client.on_message()
def handle_mqtt_message(client, userdata, message):
    #print(message.topic)
    data = dict(topic=message.topic,payload=message.payload.decode())
    print('Received message on topic: {topic} with payload: {payload}'.format(**data))
    xyz=data['payload']
    print("payload="+xyz)
    if message.topic==topic7:
        #camers_xy=r.get("global_camera_xy").split(",")
        #move_x=" X"+str(-1*int(camers_xy[0])/25)
        #move_y=" Y"+str(-1*int(camers_xy[1])/12) + " F500\r\n"
        cmd="M00 \r\n" #+move_x+move_y 
        ret=command(ser, cmd)
        time.sleep(1)
        print(cmd)
        #if ret == 'ok':
         #   pub_ret=mqtt_client.publish(topic6,"drop") # subscribe topic
    if message.topic==topic5:
        #camers_xy=r.get("global_camera_xy").split(",")
        #move_x=" X"+str(-1*int(camers_xy[0])/25)
        #move_y=" Y"+str(-1*int(camers_xy[1])/12) + " F500\r\n"
        cmd="G28 G91 X0 Y0 F500 \r\n" #+move_x+move_y 
        ret=command(ser, cmd)
        time.sleep(1)
        print(cmd)
        if ret == 'ok':
            pub_ret=mqtt_client.publish(topic6,"drop") # subscribe topic
    if message.topic==topic3:
        global pre_trackid,old_x,old_y
        xyz=data['payload']
        real_xyz=xyz.split(",")
        print(real_xyz)
        real_x=int(float(real_xyz[0]) * 1000)
        real_y=int(float(real_xyz[1]) * 1000)
        real_y=real_y-60
        real_x=real_x
        #real_x=real_x-25
        x=1*real_x
        y=1*real_y
        if not r.exists("pre_trackid"):
            r.set("pre_trackid","0")
        else:
            pre_trackid=r.get("pre_trackid")
        if pre_trackid==real_xyz[2]:
            print(" pre_trackid:"+pre_trackid)
            return
        #else:
        r.set("pre_trackid",real_xyz[2])
            #r.set("pre_trackid",real_xyz[2])
        if not r.exists("old_x"):
            r.set("old_x","0")
        else:
            old_x=r.get("old_x")
        if not r.exists("old_y"):
            r.set("old_y","0")
        else:
            old_y=r.get("old_y")
        #if abs(float(old_x)-x)<10 and abs(float(old_y)-y)<10:
         #   print("return")
         #   print(abs((float(old_x)-x)))
         #   print(abs((float(old_y)-y)))
         #   return
        #else:
        #    r.set("old_x",str(real_x))
        #    r.set("old_y",str(real_y))

        if( abs(x)> 10 or abs(y)>10) :
            print("realx,y")
            print(x)
            print(y)
            print("\n")
            #if(abs(x/50)>3 or abs(y/25)>3):
            #    return 
            #r.set("mode","moving")
            #move_x=" X"+str(x/50)
            move_x=" X"+str(x/25)
            move_y=" Y"+str(y/12) + " F500\r\n"
            cmd="G21 G91 G1 " +move_x+move_y 
            
            ret=command(ser, cmd)
            time.sleep(1)
            print(cmd,ret)
            if ret == 'ok':
                print("ret==",ret)
                r.set("global_camera_xy",""+str(x)+","+str(y))
                r.set("old_x",str(x))
                r.set("old_y",str(y))
                print(abs(x))
                print(abs(y))
                r.set("mode","pickup_ready")
                ret_ok=requests.get("http://172.26.52.69:8888/pickup")
                print(ret_ok)
                #if(abs(x)<15 and abs(y)<(15)):
                #pub_ret=mqtt_client.publish(topic4,"50") # subscribe topic
                #    print("test if send pickup ")
                 #   print("ready for pickup")
       #socketio.emit('mqtt_message', data=data)



if __name__ == '__main__':
    app.run(host='192.168.1.3', threaded=True)
