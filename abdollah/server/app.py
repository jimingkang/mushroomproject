from flask import Flask, render_template, request
import serial
import time

ser = serial.Serial("/dev/ttyACM0",115200)
def command(ser, command):
    if command:  # checks if string is empty
        print("Sending gcode:" + str(command))
        command = str.encode(command)
        ser.write(command)  # Send g-code
        return 'ok'

app = Flask(__name__)

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/annotated_camera_feed')
def annotated_camera_feed():
    return render_template('annotated_camera_feed.html')

@app.route('/depth_feed')
def depth_feed():
    return render_template('depth_feed.html')

@app.route('/automatic_scan')
def automatic_scan():
    return render_template('automatic_scan.html')

@app.route('/raw_camera_feed')
def raw_camera_feed():
    return render_template('raw_camera_feed.html')

@app.route('/move/<string:direction>')
def move(direction):
    # Open the serial port
    #ser = serial.Serial("/dev/ttyACM0", 115200, timeout=5)
    amount = 1
    x=0
    y=0
    if direction=="right":
        x=amount
    elif direction=="left":
        x=-amount
    elif direction=="up":
        y=-amount
    elif direction=="down":
        y=amount
    cmd =f'G91 G21 G1 X{x} Y{y}'
    command(ser, f"{cmd} F500\r\n")
    return cmd

if __name__ == '__main__':
    app.run(debug=True)
