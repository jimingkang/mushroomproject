#!/usr/bin/env python
#import PCA9685 as servo
import time                  # Import necessary modules

# SDA = pin.SDA_1
# SCL = pin.SCL_1
# SDA_1 = pin.SDA
# SCL_1 = pin.SCL

from adafruit_servokit import ServoKit
import board
import busio
import time


# On the Jetson Nano
# Bus 0 (pins 28,27) is board SCL_1, SDA_1 in the jetson board definition file
# Bus 1 (pins 5, 3) is board SCL, SDA in the jetson definition file
# Default is to Bus 1; We are using Bus 0, so we need to construct the busio first ...
print("Initializing Servos")
i2c_bus0=(busio.I2C(board.SCL, board.SDA))
print("Initializing ServoKit")
kit = ServoKit(channels=16, i2c=i2c_bus0,address=0x40)
# kit[0] is the bottom servo
# kit[1] is the top servo
print("Done initializing")

MinPulse = 0
MaxPulse = 180

Current_x = 0
Current_y = 0

def setup(busnum=None):
	global Xmin, Ymin, Xmax, Ymax, home_x, home_y, pwm
	offset_x = 0
	offset_y = 0
	try:
		for line in open('config'):
			if line[0:8] == 'offset_x':
				offset_x = int(line[11:-1])
				#print 'offset_x =', offset_x
			if line[0:8] == 'offset_y':
				offset_y = int(line[11:-1])
				#print 'offset_y =', offset_y
	except:
		pass
	#offset_y = -90
	Xmin = 10#MinPulse + offset_x
	Xmax = 170#MaxPulse + offset_x
	Ymin = 10#MinPulse + offset_y
	Ymax = 170#MaxPulse + offset_y
	home_x = (Xmax + Xmin)/2
	home_y = Ymin + 80
	#if busnum == None:
	#	pwm = servo.PWM()                  # Initialize the servo controller.
	#else:
	#	pwm = servo.PWM(bus_number=busnum) # Initialize the servo controller.
	#pwm.frequency = 60

# ==========================================================================================
# Control the servo connected to channel 14 of the servo control board to make the camera 
# turning towards the positive direction of the x axis.
# ==========================================================================================
def move_decrease_x(angle):
	global Current_x
	global Current_y
	Current_x +=  angle

	if Current_x > Xmax:
		Current_x = Xmax
	kit.servo[15].angle=Current_x
# ==========================================================================================
# Control the servo connected to channel 14 of the servo control board to make the camera 
# turning towards the negative direction of the x axis.
# ==========================================================================================
def move_increase_x(angle):
	global Current_x
	Current_x -=angle
	#Current_y -=50 
	if Current_x <= Xmin:
		Current_x = Xmin
	kit.servo[15].angle=Current_x

	#pwm.write(15, 0, Current_x)
	#pwm.write(12, 0, Current_y)
	#pwm.write(13, 0, Current_y)
	#pwm.write(14, 0, Current_y)
# ==========================================================================================
# Control the servo connected to channel 15 of the servo control board to make the camera 
# turning towards the positive direction of the y axis. 
# ==========================================================================================
def move_increase_y(angle):
	global Current_y
	Current_y += angle
	if Current_y > Ymax:
		Current_y = Ymax
	kit.servo[1].angle=Current_y
	kit.servo[2].angle=Current_y
	kit.servo[3].angle=Current_y
	#pwm.write(1, 0, Current_y)   # CH15 <---> Y axis
	#pwm.write(4, 0, Current_y)   # CH15 <---> Y axis
	#pwm.write(3,0, Current_y)   # CH15 <---> Y axis
# ==========================================================================================
# Control the servo connected to channel 15 of the servo control board to make the camera 
# turning towards the negative direction of the y axis. 
# ==========================================================================================		
def move_decrease_y(angle):
	global Current_y
	Current_y -= angle
	if Current_y <= Ymin:
		Current_y = Ymin
	kit.servo[1].angle=Current_y
	kit.servo[2].angle=Current_y
	kit.servo[3].angle=Current_y
	#pwm.write(1, 0, Current_y)
	#pwm.write(4, 0, Current_y)
	#pwm.write(3, 0, Current_y)
# ==========================================================================================		
# Control the servos connected with channel 14 and 15 at the same time to make the camera 
# move forward.
# ==========================================================================================
def home_x_y():
	global Current_y
	global Current_x
	Current_y =  90
	Current_x = 90
	kit.servo[1].angle=90
	kit.servo[2].angle=90
	kit.servo[3].angle=90
	#pwm.write(4, 0, Current_y)
	#pwm.write(1, 0, Current_y)
	#pwm.write(2, 0, Current_y)

def calibrate(x,y):
	pwm.write(11, 0, (MaxPulse+MinPulse)/2+x)
	pwm.write(15, 0, (MaxPulse+MinPulse)/2+y)

def test():
	while True:
		home_x_y()
		time.sleep(0.5)
		for i in range(0, 9):
			move_increase_x()
			move_increase_y()
			time.sleep(0.5)
		for i in range(0, 9):
			move_decrease_x()
			move_decrease_y()
			time.sleep(0.5)

if __name__ == '__main__':
	setup()
	home_x_y()

=======
#!/usr/bin/env python
import PCA9685 as servo
import time                  # Import necessary modules

MinPulse = 200
MaxPulse = 700

Current_x = 0
Current_y = 0

def setup(busnum=None):
	global Xmin, Ymin, Xmax, Ymax, home_x, home_y, pwm
	offset_x = 0
	offset_y = 0
	try:
		for line in open('config'):
			if line[0:8] == 'offset_x':
				offset_x = int(line[11:-1])
				#print 'offset_x =', offset_x
			if line[0:8] == 'offset_y':
				offset_y = int(line[11:-1])
				#print 'offset_y =', offset_y
	except:
		pass
	offset_y = -90
	Xmin = MinPulse + offset_x
	Xmax = MaxPulse + offset_x
	Ymin = MinPulse + offset_y
	Ymax = MaxPulse + offset_y
	home_x = (Xmax + Xmin)/2
	home_y = Ymin + 80
	if busnum == None:
		pwm = servo.PWM()                  # Initialize the servo controller.
	else:
		pwm = servo.PWM(bus_number=busnum) # Initialize the servo controller.
	pwm.frequency = 60

# ==========================================================================================
# Control the servo connected to channel 14 of the servo control board to make the camera 
# turning towards the positive direction of the x axis.
# ==========================================================================================
def move_decrease_x(angle):
	global Current_x
	global Current_y
	Current_x +=  angle
	#Current_y +=50 
	if Current_x > Xmax:
		Current_x = Xmax
	pwm.write(15, 0, Current_x)   # CH14 <---> X axis
	#pwm.write(12, 0, Current_y)
	#pwm.write(13, 0, Current_y)
	#pwm.write(14, 0, Current_y)
# ==========================================================================================
# Control the servo connected to channel 14 of the servo control board to make the camera 
# turning towards the negative direction of the x axis.
# ==========================================================================================
def move_increase_x(angle):
	global Current_x
	Current_x -=angle
	#Current_y -=50 
	if Current_x <= Xmin:
		Current_x = Xmin
	pwm.write(15, 0, Current_x)
	#pwm.write(12, 0, Current_y)
	#pwm.write(13, 0, Current_y)
	#pwm.write(14, 0, Current_y)
# ==========================================================================================
# Control the servo connected to channel 15 of the servo control board to make the camera 
# turning towards the positive direction of the y axis. 
# ==========================================================================================
def move_increase_y(angle):
	global Current_y
	Current_y += angle
	if Current_y > Ymax:
		Current_y = Ymax
	pwm.write(2, 0, Current_y)   # CH15 <---> Y axis
	pwm.write(4, 0, Current_y)   # CH15 <---> Y axis
	pwm.write(5,0, Current_y)   # CH15 <---> Y axis
# ==========================================================================================
# Control the servo connected to channel 15 of the servo control board to make the camera 
# turning towards the negative direction of the y axis. 
# ==========================================================================================		
def move_decrease_y(angle):
	global Current_y
	Current_y -= angle
	if Current_y <= Ymin:
		Current_y = Ymin
	pwm.write(2, 0, Current_y)
	pwm.write(4, 0, Current_y)
	pwm.write(5, 0, Current_y)
# ==========================================================================================		
# Control the servos connected with channel 14 and 15 at the same time to make the camera 
# move forward.
# ==========================================================================================
def home_x_y():
	global Current_y
	global Current_x
	Current_y = home_y 
	Current_x = home_x
	pwm.write(4, 0, Current_y)
	pwm.write(1, 0, Current_y)
	pwm.write(2, 0, Current_y)

def calibrate(x,y):
	pwm.write(11, 0, (MaxPulse+MinPulse)/2+x)
	pwm.write(15, 0, (MaxPulse+MinPulse)/2+y)

def test():
	while True:
		home_x_y()
		time.sleep(0.5)
		for i in range(0, 9):
			move_increase_x()
			move_increase_y()
			time.sleep(0.5)
		for i in range(0, 9):
			move_decrease_x()
			move_decrease_y()
			time.sleep(0.5)

if __name__ == '__main__':
	setup()
	home_x_y()
