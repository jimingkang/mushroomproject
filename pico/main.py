from ifconfig import *
do_connect()
#import upip
#upip.install("micropython-phew")

from phew import logging, server, connect_to_wifi
from phew.template import render_template


#connect_to_wifi(ssid, password)

import time
from machine import Pin, PWM
from robot_car import RobotCar
import utime

# Pico W GPIO Pin
LEFT_MOTOR_PIN_1 = 16
LEFT_MOTOR_PIN_2 = 17
RIGHT_MOTOR_PIN_1 = 18
RIGHT_MOTOR_PIN_2 = 19

motor_pins = [LEFT_MOTOR_PIN_1, LEFT_MOTOR_PIN_2, RIGHT_MOTOR_PIN_1, RIGHT_MOTOR_PIN_2]
robot_car = RobotCar(motor_pins, 20000)

MIN_DUTY = 200 # 5 percent of 65025 = 3251.25
MAX_DUTY = 700 # 10 percent of 65025 = 6502.5

pwm = PWM(Pin(0))
pwm.freq(50)

duty = MIN_DUTY
direction = 1
i=0
@server.route("/")
def index(request):
    return render_template("index.html", name="jimmy")
@server.route("/forward")
def index(request):
    print("Moving forward")
    robot_car.move_forward()
    return render_template("index.html", name="forward")
@server.route("/backward")
def index(request):
    print("/backward")
    robot_car.move_backward()
    #i=0
    #global duty
    #duty -= 25
    #if duty > MAX_DUTY:
    #    duty = MAX_DUTY
    #elif duty < MIN_DUTY:
    #    duty= MIN_DUTY
    #pwm.duty_u16(duty)
    return render_template("index.html", name="backward")

@server.route("/stop")
def index(request):
    print("/stop")
    robot_car.stop()
    #i=0
    #global duty
    #duty -= 25
    #if duty > MAX_DUTY:
    #    duty = MAX_DUTY
    #elif duty < MIN_DUTY:
    #    duty= MIN_DUTY
    #pwm.duty_u16(duty)
    return render_template("index.html", name="stop")
server.run()