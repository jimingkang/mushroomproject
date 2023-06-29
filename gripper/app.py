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
            broker = line[9:-1]
except:
    pass
 
app = Flask(__name__)
#app.config['MQTT_BROKER_URL'] = broker
app.config['MQTT_BROKER_URL'] = '192.168.254.42'
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

def command(ser, command):
  return ser.write(str.encode(command)) 
  #time.sleep(1)


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

@mqtt_client.on_message()
def handle_mqtt_message(client, userdata, message):
    print('Received message on topic:')
    print(message.topic)
    if message.topic==topic3:
       data = dict(
           topic=message.topic,
           payload=message.payload.decode()
      )
       print('Received message on topic: {topic} with payload: {payload}'.format(**data))
       xyz=data['payload']
       print("payload="+xyz)
       ret=command(ser, xyz)
       print("ret,",ret)
       if ret == 22:
            mqtt_client.publish(topic4,"") # subscribe topic
       #socketio.emit('mqtt_message', data=data)

@app.route('/publish', methods=['POST'])
def publish_message():
   request_data = request.get_json()
   publish_result = mqtt_client.publish(request_data['topic'], request_data['msg'])
   return jsonify({'code': publish_result[0]})
 
if __name__ == '__main__':
    app.run()
