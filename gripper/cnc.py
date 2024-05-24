from threading import Event

from flask import Flask, render_template,request, jsonify
from flask_mqtt import Mqtt
#from flask_socketio import SocketIO
#import video_dir
#import car_dir
#import motor
import redis
import serial
import datetime
import time
import os
from time import sleep



ser = serial.Serial("/dev/ttyACM0",115200)

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

 
if __name__ == '__main__':
    #app.run(host='0.0.0.0',port=8888)
    x=35
    y=40
    move_x=" X"+str(x/35)
    move_y=" Y"+str(y/45) + " F100\r\n"
    cmd="G21 G91 G1 " +move_x+move_y 
    ret=command(ser, cmd)

