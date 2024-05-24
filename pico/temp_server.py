import machine
import socket
import math
import utime
import network
import time
from machine import Pin, PWM

MIN_DUTY = 1000 # 5 percent of 65025 = 3251.25
MAX_DUTY = 9000 # 10 percent of 65025 = 6502.5

pwm = PWM(Pin(0))
pwm.freq(50)
duty = MIN_DUTY
direction = 1
 
wlan = network.WLAN(network.STA_IF)
wlan.active(True)
wlan.connect("jimmy2041","Nokia123")
 
# rgb led
red=machine.Pin(13,machine.Pin.OUT)
green=machine.Pin(14,machine.Pin.OUT)
blue=machine.Pin(15,machine.Pin.OUT)
 
# Wait for connect or fail
wait = 10
while wait > 0:
    if wlan.status() < 0 or wlan.status() >= 3:
        break
    wait -= 1
    print('waiting for connection...')
    time.sleep(1)
 
# Handle connection error
if wlan.status() != 3:
    raise RuntimeError('wifi connection failed')
else:
    print('connected')
    ip=wlan.ifconfig()[0]
    print('IP: ', ip)
 
# Temperature Sensor
sensor_temp = machine.ADC(4)
conversion_factor = 3.3 / (65535)
 
def temperature():
    temperature_value = sensor_temp.read_u16() * conversion_factor 
    temperature_Celcius = 27 - (temperature_value - 0.706)/0.00172169/ 8 
    print(temperature_Celcius)
    utime.sleep(2)
    return temperature_Celcius
 
def webpage(value):
    html = f"""
            <!DOCTYPE html>
            <html>
            <body>
            <form action="./forward">
            <input type="submit" value="forward " />
            </form>
            <form action="./backward">
            <input type="submit" value="backward" />
            </form>
            <form action="./blue">
            <input type="submit" value="blue" />
            </form>
            <form action="./off">
            <input type="submit" value="off" />
            </form>
            <p>Temperature is {value} degrees Celsius</p>
            </body>
            </html>
            """
    return html
def backward():
    print("test /backward")
    i=0
    global duty
    duty -= 25
    if duty > MAX_DUTY:
        duty = MAX_DUTY
    elif duty < MIN_DUTY:
        duty= MIN_DUTY
    pwm.duty_u16(duty)

def forward():
    print("test /forward")
    i=0
    global duty
    duty += 25
    if duty > MAX_DUTY:
        duty = MAX_DUTY
    elif duty < MIN_DUTY:
        duty= MIN_DUTY
    pwm.duty_u16(duty)
    #return render_template("index.html", name="jimmy")
def serve(connection):
    while True:
        client = connection.accept()[0]
        request = client.recv(1024)
        request = str(request)
        try:
            request = request.split()[1]
        except IndexError:
            pass
        
        print(request)
        
        if request == '/backward?':
            backward()
        elif request == '/forward?':
            forward()
        elif request == '/green?':
            red.low()
            green.high()
            blue.low()
        elif request == '/blue?':
            red.low()
            green.low()
            blue.high()
 
        value='%.2f'%temperature()    
        html=webpage(value)
        client.send(html)
        client.close()
 
def open_socket(ip):
    # Open a socket
    address = (ip, 80)
    connection = socket.socket()
    connection.bind(address)
    connection.listen(1)
    print(connection)
    return(connection)
 
 
try:
    if ip is not None:
        connection=open_socket(ip)
        serve(connection)
except KeyboardInterrupt:
    machine.reset()