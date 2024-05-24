from threading import Event

from flask import Flask, render_template,request, jsonify
from flask_mqtt import Mqtt
#from flask_socketio import SocketIO
import video_dir
#import car_dir
#import motor
import redis
import serial
import datetime
import time
import os
from time import sleep

#import config

busnum = 0          # Edit busnum to 0, if you uses Raspberry Pi 1 or 0

video_dir.setup(busnum=busnum)
#motor.setup(busnum=busnum)     # Initialize the Raspberry Pi GPIO connected to the DC motor. 
#motor.setSpeed(40)
video_dir.home_x_y()

i=0


#ser = serial.Serial("/dev/ttyACM0",115200)

redis_server=''
broker=''
try:
    for line in open("../ip.txt"):
        if line[0:6] == "broker":
            broker = line[9:len(line)-1]
        if line[0:6] == "reddis":
            redis_server = line[9:len(line)-1]
except:
    pass
broker=broker.replace("\n","").replace("\r\n","")
print(broker)
pool = redis.ConnectionPool(host=redis_server, port=6379, decode_responses=True,password='jimmy')
r = redis.Redis(connection_pool=pool)
app = Flask(__name__)
app.config['MQTT_BROKER_URL'] = broker
app.config['MQTT_BROKER_PORT'] = 1883
app.config['MQTT_USERNAME'] = ''  # Set this item when you need to verify username and password
app.config['MQTT_PASSWORD'] = ''  # Set this item when you need to verify username and password
app.config['MQTT_KEEPALIVE'] = 5  # Set KeepAlive time in seconds
app.config['MQTT_TLS_ENABLED'] = False  # If your server supports TLS, set it True
#socketio = SocketIO(app)
topic = '/flask/scan'
topic2 = '/flask/xyz'
topic3 = '/flask/serial'
topic4 = '/flask/pickup'
topic5 = '/flask/collection'
topic6 = '/flask/drop'
mqtt_client = Mqtt(app)
y=0
#app.config.from_object(config)


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


@app.route('/test')
def  test():
    #video_dir.move_decrease_y()
    return render_template('index.html');

@app.route('/catch')
def catch():
    video_dir.move_increase_y(10)
    #motor.forward()
    #time.sleep(0.5)
    #motor.ctrl(0)
    print("catch")
    return render_template('index.html');

@app.route('/release')
def  release():
    video_dir.move_decrease_y(10)
    #motor.backward()
    #time.sleep(0.5)
    #motor.ctrl(0)
    return render_template('index.html');

@app.route('/autoscan')
def  autopick():
    publish_result = mqtt_client.publish(topic, "/flask/scan")
    command(ser, "G21 G91 G1 X1 F500\r\n")
    return render_template('index.html');

@app.route('/home')
def  home():
    publish_result = mqtt_client.publish(topic, "/flask/home")
    #command(ser, "G28 G91  X0 Y0 F500\r\n")
    return render_template('index.html');



@app.route('/forward')
def forward():
    #move_forward();
    video_dir.move_decrease_x(30)
    return render_template('index.html');
@app.route('/backward')
def backward():
    #move_backward();
    video_dir.move_increase_x(30)
    return render_template('index.html');

#83.3
def move_forward():
    #Moving forward code
    command(ser, "G21 G91 G1 Y-3 F500\r\n")
    #video_dir.move_decrease_y()
    return 'Moving Forward...!'
def move_backward():
    command(ser, "G21 G91 G1  Y3 F500\r\n")
    #video_dir.move_increase_y()
    return 'Moving Backward...!'


@app.route("/drop")
def drop():
    if 1:#r.get("mode")=="pickup_ready":
        time.sleep(1)
        video_dir.move_decrease_y(150)
        r.set("mode","drop_ready")
        return "OK"
@app.route("/pickup")
def pickup():
    if 1:#r.get("mode")=="pickup_ready":
        video_dir.move_increase_x(50)
        time.sleep(1)
        video_dir.move_increase_y(150)
        time.sleep(1)
        video_dir.move_decrease_x(50)
        time.sleep(1)
        video_dir.move_decrease_y(150)
        r.set("mode","drop_ready")
        return "OK"

 
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
       mqtt_client.subscribe(topic4) # subscribe topic
       #mqtt_client.subscribe(topic6) # subscribe topic
   else:
       print('Bad connection. Code:', rc)

pre_trackid=0
old_x=0
old_y=0
@mqtt_client.on_message()
def handle_mqtt_message(client, userdata, message):
    #print(message.topic)
    if message.topic==topic6:
        data = dict(topic=message.topic,payload=message.payload.decode())
        print('Received drop message on topic: {topic} with payload: {payload}'.format(**data))
        collection=data['payload']
        print("payload="+collection)
        #real_z=xyz.split(",")
        video_dir.move_increase_x()
        time.sleep(1)
        #r.set("mode","camera_ready")
    if message.topic==topic4:
        data = dict(topic=message.topic,payload=message.payload.decode())
        print('Received message on topic: {topic} with payload: {payload}'.format(**data))
        xyz=data['payload']
        print("payload="+xyz)
        #real_z=xyz.split(",")
        real_z=xyz
        print(real_z)
        time.sleep(2)
        video_dir.move_increase_x()
        #time.sleep(2)
        video_dir.move_increase_y(50)
        time.sleep(1)
        video_dir.move_decrease_x()
        time.sleep(1)
        r.set("mode","camera_ready")
        video_dir.move_decrease_y(50)
        time.sleep(1)
        #pub_ret=mqtt_client.publish(topic5,"collection") # subscribe topic
       #socketio.emit('mqtt_message', data=data)

@app.route('/publish', methods=['POST'])
def publish_message():
   request_data = request.get_json()
   publish_result = mqtt_client.publish(request_data['topic'], request_data['msg'])
   return jsonify({'code': publish_result[0]})
 
if __name__ == '__main__':
    app.run(host='0.0.0.0',port=8888)
