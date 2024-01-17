from threading import Event
import requests
from flask import Flask, render_template, request, jsonify
from flask_mqtt import Mqtt
# from flask_socketio import SocketIO
import video_dir
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

<<<<<<< HEAD
topic = "/flask/stop"
=======
topic = '/flask/scan'
topic2 = '/flask/xyz'
topic3 = '/flask/serial'
topic4 = '/flask/pickup'
topic5 = '/flask/home'
topic6 = '/flask/drop'
topic7 = '/flask/stop'
pre_trackid=0
old_x=0
old_y=0

>>>>>>> 2f9c2b9b5f4ee0211feb6c12d088236efcbe32d0
# Pin Definitions
input_pin = 12  # BCM pin 18, BOARD pin 12
busnum = 1          # Edit busnum to 0, if you uses Raspberry Pi 1 or 0
y=0
i=0
prev_value = None
<<<<<<< HEAD
# GPIO.setmode(GPIO.BOARD)  # BCM pin-numbering scheme from Raspberry Pi
# GPIO.setup(input_pin, GPIO.IN)  # set pin as an input pin


# import config

busnum = 1  # Edit busnum to 0, if you uses Raspberry Pi 1 or 0

# video_dir.setup(busnum=busnum)
# motor.setup(busnum=busnum)     # Initialize the Raspberry Pi GPIO connected to the DC motor.
# motor.setSpeed(40)
# video_dir.home_x_y()

i = 0
=======






#import config
GPIO.setmode(GPIO.BOARD)  # BCM pin-numbering scheme from Raspberry Pi
GPIO.setup(input_pin, GPIO.IN)  # set pin as an input pin
video_dir.setup(busnum=busnum)
#motor.setup(busnum=busnum)     # Initialize the Raspberry Pi GPIO connected to the DC motor. 
#motor.setSpeed(40)
video_dir.home_x_y()


>>>>>>> 2f9c2b9b5f4ee0211feb6c12d088236efcbe32d0

ser = serial.Serial("/dev/ttyACM0", 115200)

<<<<<<< HEAD
redis_server = ''
broker = ''
=======
ser = serial.Serial("/dev/ttyACM0",115200)

redis_server=''
broker=''
>>>>>>> 2f9c2b9b5f4ee0211feb6c12d088236efcbe32d0
try:
    for line in open("../ip.txt"):
        if line[0:6] == "broker":
            broker = line[9:len(line) - 1]
        if line[0:6] == "reddis":
            redis_server = line[9:len(line) - 1]
except:
    pass
#broker = broker.replace("\n", "").replace("\r\n", "")
broker = "192.168.1.3"
print(broker)
<<<<<<< HEAD
print(redis_server)
pool = redis.ConnectionPool(host=redis_server, port=6379, decode_responses=True, password='jimmy')
=======

pool = redis.ConnectionPool(host=redis_server, port=6379, decode_responses=True,password='jimmy')
>>>>>>> 2f9c2b9b5f4ee0211feb6c12d088236efcbe32d0
r = redis.Redis(connection_pool=pool)
app = Flask(__name__)
app.config['MQTT_BROKER_URL'] = broker
# app.config['MQTT_BROKER_URL'] = '10.0.0.134'
# app.config['MQTT_BROKER_URL'] = '192.168.254.42'
app.config['MQTT_BROKER_PORT'] = 1883
app.config['MQTT_USERNAME'] = ''  # Set this item when you need to verify username and password
app.config['MQTT_PASSWORD'] = ''  # Set this item when you need to verify username and password
app.config['MQTT_KEEPALIVE'] = 5  # Set KeepAlive time in seconds
app.config['MQTT_TLS_ENABLED'] = False  # If your server supports TLS, set it True
<<<<<<< HEAD
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
=======
#socketio = SocketIO(app)
mqtt_client = Mqtt(app)

#app.config.from_object(config)
>>>>>>> 2f9c2b9b5f4ee0211feb6c12d088236efcbe32d0


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
    # video_dir.move_decrease_y()
    r.set("mode", "camera_ready")
    #r.set("global_camera_xy","0,0")
    r.delete("detections")
    loop_pickup();
    return render_template('index.html');


@app.route('/down')
def down():
    command(ser, "$X\r\n")
    time.sleep(1)
    command(ser, "G21 G91 G1 X1 F500\r\n")
    xy = r.get("global_camera_xy").split(",")
    print(xy)
    r.set("global_camera_xy", str(int(xy[0]) - 25) + "," + xy[1])
    return render_template('index.html');


@app.route('/up')
def up():
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
    # motor.backward()
    # time.sleep(0.5)
    # motor.ctrl(0)
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


@app.route('/forward')
def forward():
    move_forward();
    # video_dir.move_decrease_x()
    return render_template('index.html');


@app.route('/backward')
def backward():
    move_backward();
    # video_dir.move_increase_x()
    return render_template('index.html');


# 83.3
def move_forward():
    command(ser, "$X\r\n")
    time.sleep(1)
    # Moving forward code
    command(ser, "G21 G91 G1  Y-5 F500\r\n")
    xy = r.get("global_camera_xy").split(",")
    print(xy)
    r.set("global_camera_xy", xy[0] + "," + str(int(xy[1]) + 60))
    # video_dir.move_decrease_y()
    return 'Moving Forward...!'


def move_backward():
    command(ser, "$X\r\n")
    time.sleep(1)
    command(ser, "G21 G91 G1  Y5 F500\r\n")
    xy = r.get("global_camera_xy").split(",")
    r.set("global_camera_xy", xy[0] + "," + str(int(xy[1]) - 60))
    # video_dir.move_increase_y()
    return 'Moving Backward...!'


@app.route('/')
def index():
    return render_template('index.html')


@app.route('/msg')
def msg():
    return render_template('index.html')


@mqtt_client.on_connect()
def handle_connect(client, userdata, flags, rc):
    if rc == 0:
        print('Connected successfully')
<<<<<<< HEAD
        mqtt_client.subscribe(topic3)  # subscribe topic
        # mqtt_client.subscribe(topic5) # subscribe topic
    else:
        print('Bad connection. Code:', rc)


pre_trackid = 0
old_x = 0
old_y = 0

=======
        mqtt_client.subscribe(topic3) # subscribe topic
        #mqtt_client.subscribe(topic5) # subscribe topic
   else:
       print('Bad connection. Code:', rc)
>>>>>>> 2f9c2b9b5f4ee0211feb6c12d088236efcbe32d0

@mqtt_client.on_message()
def handle_mqtt_message(client, userdata, message):
    # print(message.topic)
    data = dict(topic=message.topic, payload=message.payload.decode())
    print('Received message on topic: {topic} with payload: {payload}'.format(**data))
    xyz = data['payload']
    print("get xy payload=" + xyz)
    if message.topic == topic7:
        # camers_xy=r.get("global_camera_xy").split(",")
        # move_x=" X"+str(-1*int(camers_xy[0])/25)
        # move_y=" Y"+str(-1*int(camers_xy[1])/12) + " F500\r\n"
        cmd = "M00 \r\n"  # +move_x+move_y
        ret = command(ser, cmd)
        time.sleep(1)
        print(cmd)
        # if ret == 'ok':
        #   pub_ret=mqtt_client.publish(topic6,"drop") # subscribe topic
    if message.topic == topic5:
        # camers_xy=r.get("global_camera_xy").split(",")
        # move_x=" X"+str(-1*int(camers_xy[0])/25)
        # move_y=" Y"+str(-1*int(camers_xy[1])/12) + " F500\r\n"
        cmd = "G28 G91 X0 Y0 F500 \r\n"  # +move_x+move_y
        ret = command(ser, cmd)
        time.sleep(1)
        print(cmd)
        if ret == 'ok':
            pub_ret = mqtt_client.publish(topic6, "drop")  # subscribe topic
    if message.topic == topic3:
        global pre_trackid, old_x, old_y
        xyz = data['payload']
        if xyz=="0.1,0.1,1":
            return
        print(xyz)
        real_xyz = xyz.split(",")
        real_xyz = xyz.split(",")
        # detections=r.hgetall("detections")
        # for item in r.hkeys("detections"):
        #    real_xyz=r.hget("detections",item).split(",")
        # real_xyz=detections[0].split(",")
        real_x = int(float(real_xyz[0]) * 1000)
        real_y = int(float(real_xyz[1]) * 1000)
        real_y = real_y - 190
        real_x = real_x + 10
        # real_x=real_x-25
        x = -1 * real_x
        y = -1 * real_y
        if not r.exists("pre_trackid"):
            r.set("pre_trackid", "0")
        else:
            pre_trackid = r.get("pre_trackid")
        #if pre_trackid == real_xyz[2]:
        #    print(" pre_trackid:" + pre_trackid)
        #    return
       # else:
       #     r.set("pre_trackid", real_xyz[2])
        # r.set("pre_trackid",real_xyz[2])
        if not r.exists("old_x"):
            r.set("old_x", "0")
        else:
            old_x = r.get("old_x")
        if not r.exists("old_y"):
            r.set("old_y", "0")
        else:
            old_y = r.get("old_y")
        if abs(float(old_x)-x)<10 and abs(float(old_y)-y)<10:
           print("return")
           print(abs((float(old_x)-x)))
           print(abs((float(old_y)-y)))
           return
        else:
            r.set("old_x",str(real_x))
            r.set("old_y",str(real_y))

        if (abs(x) > 10 or abs(y) > 10):
            r.set("mode", "pickup_ready")
            print("realx,y")
            print(x)
            print(y)
            print("\n")
            #if(abs(x/50)>3 or abs(y/25)>3):
             #   return
            # r.set("mode","moving")
            # move_x=" X"+str(x/50)
            command(ser, "$X\r\n")
            time.sleep(1)
            move_x = " X" + str(x / 25)
            move_y = " Y" + str(y / 12) + " F500\r\n"
            cmd = "G21 G91 G1 " + move_x + move_y

            ret = command(ser, cmd)
            time.sleep(1)
            print(cmd, ret)
            if ret == 'ok':
                print("ret==", ret)
                r.set("global_camera_xy", "" + str(x) + "," + str(y))
                r.set("old_x", str(x))
                r.set("old_y", str(y))
                print(abs(x))
                print(abs(y))
                ret_ok = requests.get("http://192.168.1.3:8888/pickup")
                time.sleep(1)
                #ret_ok = requests.get("http://172.26.52.51:8888/pickup")
                #command(ser, "$X\r\n")
                #time.sleep(1)
                #command(ser, "G28 G91 X0.0 Y0.00 F500\r\n")
                print(ret_ok)
                #ret_ok = requests.get("http://172.26.52.51:8888/drop")
                r.set("mode", "camera_ready")
                # if(abs(x)<15 and abs(y)<(15)):
                # pub_ret=mqtt_client.publish(topic4,"50") # subscribe topic
                #    print("test if send pickup ")
                #   print("ready for pickup")
    # socketio.emit('mqtt_message', data=data)


@app.route('/publish', methods=['POST'])
def publish_message():
    request_data = request.get_json()
    publish_result = mqtt_client.publish(request_data['topic'], request_data['msg'])
    return jsonify({'code': publish_result[0]})


# @app.route('/autoscan')
def loop_pickup():
    camera_xy = r.get("global_camera_xy").split(",")
    command(ser, "$X\r\n")
    time.sleep(1)
    command(ser, "G21 G91 X"+str(int(camera_xy[0])/25)+" Y"+str(int(camera_xy[1])/12)+" F500\r\n")
    while (1):
        detections = r.hgetall("detections")
        for item in r.hkeys("detections"):
            r.set("mode", "pickup_ready")
            real_xyz = r.hget("detections", item).split(",")
            real_x = int(float(real_xyz[0]))
            real_y = int(float(real_xyz[1]))
            camera_xy = r.get("global_camera_xy").split(",")
            real_y = real_y - 190  # -int(camera_xy[0])
            real_x = real_x +0  # -int(camera_xy[1])
            x = -1 * real_x
            y = -1 * real_y
            if (abs(x) > 10 or abs(y) > 10):
                print("realx,y")
                print(x)
                print(y)
                print("\n")
                command(ser, "$X\r\n")
                time.sleep(1)
                move_x = " X" + str(x / 25)
                move_y = " Y" + str(y / 12) + " F500\r\n"
                cmd = "G21 G91 G1 " + move_x + move_y

                ret = command(ser, cmd)
                time.sleep(1)
                print(cmd, ret)
                if ret == 'ok':
                    #r.set("global_camera_xy", str(int(camera_xy[0]) + real_x) + "," + str(int(camera_xy[1]) + real_y))
                    r.hdel("detections", item)
                    print("ret==", ret)
                    print(abs(x))
                    print(abs(y))
                    ret_ok = requests.get("http://172.26.52.51:8888/pickup")
                    time.sleep(1)
                    ret_ok = requests.get("http://172.26.52.51:8888/pickup")
                    command(ser, "$X\r\n")
                    time.sleep(1)
                    move_x = " X" + str(-x / 25)
                    move_y = " Y" + str(-y / 12) + " F500\r\n"
                    cmd = "G21 G91 G1 " + move_x + move_y
                    ret = command(ser, cmd)
                    time.sleep(1)
                    print(ret_ok)
                    ret_ok = requests.get("http://172.26.52.51:8888/drop")
        r.set("mode", "camera_ready")
        time.sleep(2)




if __name__ == '__main__':
    app.run(host='0.0.0.0', port=8888)

