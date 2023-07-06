from threading import Event

from flask import Flask, render_template,request, jsonify
from flask_mqtt import Mqtt
#from flask_socketio import SocketIO
#import video_dir
#import car_dir
#import motor

import serial
import datetime
import time
import os
from time import sleep

#import config

busnum = 1          # Edit busnum to 0, if you uses Raspberry Pi 1 or 0

#video_dir.setup(busnum=busnum)
#motor.setup(busnum=busnum)     # Initialize the Raspberry Pi GPIO connected to the DC motor. 
#motor.setSpeed(40)
#video_dir.home_x_y()

i=0


ser = serial.Serial("/dev/ttyACM0",115200)

broker=''
try:
    for line in open("ip.txt"):
        if line[0:6] == "broker":
            broker = line[9:len(line)]
except:
    pass
broker=broker.replace("\n","").replace("\r\n","")
print(broker)
app = Flask(__name__)
app.config['MQTT_BROKER_URL'] = broker
#app.config['MQTT_BROKER_URL'] = '10.0.0.134'
#app.config['MQTT_BROKER_URL'] = '192.168.254.42'
app.config['MQTT_BROKER_PORT'] = 1883
app.config['MQTT_USERNAME'] = ''  # Set this item when you need to verify username and password
app.config['MQTT_PASSWORD'] = ''  # Set this item when you need to verify username and password
app.config['MQTT_KEEPALIVE'] = 5  # Set KeepAlive time in seconds
app.config['MQTT_TLS_ENABLED'] = False  # If your server supports TLS, set it True
#socketio = SocketIO(app)
topic = '/flask/scan'
topic2 = '/flask/xyz'
topic3 = '/flask/serial'
topic4 = '/flask/downmove'
mqtt_client = Mqtt(app)
y=0
#app.config.from_object(config)


pub_ret=mqtt_client.publish(topic4,"22",qos=0) # subscribe topic
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
    send_wake_up(ser)
    if command:  # checks if string is empty
        print("Sending gcode:" + str(command))
        # converts string to byte encoded string and append newline
        command = str.encode(command)
        #command = str.encode(command + x'\r\n')
        ser.write(command)  # Send g-code
        #wait_for_movement_completion(ser, command)
        #grbl_out = ser.readline()  # Wait for response with carriage return
        #print(" : ", grbl_out.strip().decode('utf-8'))
        #time.sleep(2)
    return 'ok'
    #return grbl_out.strip().decode('utf-8')


@app.route('/test')
def  test():
    #video_dir.move_decrease_y()
    return render_template('index.html');

@app.route('/catch')
def catch():
    #video_dir.move_increase_y()
    #motor.forward()
    #time.sleep(0.5)
    #motor.ctrl(0)
    print("catch")
    return render_template('index.html');

@app.route('/release')
def  release():
    #video_dir.move_decrease_y()
    #motor.backward()
    #time.sleep(0.5)
    #motor.ctrl(0)
    return render_template('index.html');

@app.route('/autoscan')
def  autopick():
    publish_result = mqtt_client.publish(topic, "flask/scan")
    command(ser, "G21 G91 G1 X100 F2540\r\n")
    return render_template('index.html');




@app.route('/forward')
def forward():
    move_forward();
    return render_template('index.html');
@app.route('/backward')
def backward():
    move_backward();
    return render_template('index.html');

#83.3
def move_forward():
    #Moving forward code
    command(ser, "G21 G91 G1 Y-30 F2540\r\n")
    #video_dir.move_decrease_y()
    return 'Moving Forward...!'
def move_backward():
    command(ser, "G21 G91 G1  Y30 F2540\r\n")
    #video_dir.move_increase_y()
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
       mqtt_client.subscribe(topic3) # subscribe topic
   else:
       print('Bad connection. Code:', rc)

global pre_time
@mqtt_client.on_message()
def handle_mqtt_message(client, userdata, message):
    #print(message.topic)
    if message.topic==topic3:
        #global pre_time
        #nonlocal pre_time
        #curr_time = round(time.time() * 1000)
        #time_diff = curr_time - pre_time
        #pre_time = curr_time
        print('Received message on topic time:', time.time())
        #if time_diff < 1000:
        #    return
        data = dict(topic=message.topic,payload=message.payload.decode())
        print('Received message on topic: {topic} with payload: {payload}'.format(**data))
        xyz=data['payload']
        print("payload="+xyz)
        ret=command(ser, xyz)
        time.sleep(1)
        #ret='ok'#command(ser, xyz)
        #print(ret)
        if ret == 'ok':
            #print("ret==",ret)
            pub_ret=mqtt_client.publish(topic4,"22",qos=0) # subscribe topic
       #socketio.emit('mqtt_message', data=data)

@app.route('/publish', methods=['POST'])
def publish_message():
   request_data = request.get_json()
   publish_result = mqtt_client.publish(request_data['topic'], request_data['msg'])
   return jsonify({'code': publish_result[0]})
 
if __name__ == '__main__':
    app.run()
