#!home/gripper_ws/venv/bin/python
import rclpy
from rclpy.node import Node
from rclpy.timer import Timer
from std_msgs.msg import String
from example_interfaces.srv import Trigger  # Import the Trigger service
import time
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
import busio
import board
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
from rclpy.duration import Duration
from std_msgs.msg import Int32

class TimerNode(Node):
    def __init__(self):
        super().__init__('timer_node') 
        self.timer = self.create_timer(0.02, self.timer_callback)
        self.publisher = self.create_publisher(String, 'timer_topic', 10)
        self.wait_time=30
        # I2C setup for sensor
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.ads = ADS.ADS1115(self.i2c)
        self.chan = AnalogIn(self.ads, ADS.P0)
        self.value = 0
        self.pca = PCA9685(self.i2c)
        self.pca.frequency = 60  # Set PWM frequency for servo control
         
        # Create a servo object on the first pin (pin 0)
        self.servo1 = servo.Servo(self.pca.channels[0])
        self.servo2 = servo.Servo(self.pca.channels[1])
        self.servo3 = servo.Servo(self.pca.channels[3])
        self.servo_yaw= servo.Servo(self.pca.channels[6])
        self.servo_roll= servo.Servo(self.pca.channels[7])
        self.yaw_angle=90
        self.servo_roll.angle=90
        self.servo_yaw.angle=self.yaw_angle
        # Create the service with an empty trigger
        self.srv_gripper_close = self.create_service(Trigger, 'close_gripper_service', self.close_gripper_service_callback)
        self.srv_gripper_open = self.create_service(Trigger, 'open_gripper_service', self.open_gripper_service_callback)
        
        self.subscription = self.create_subscription(
            Int32,
            '/gripper_angle',
            self.callback_set_gripper_angle,
            10
        )
        
        self.close_gripper=True
        self.rotation=False
        self.rotation_trigger=True
        self.angle=90
        self.close_gripper_start_time = self.get_clock().now()
        self.postive_rotation=False
        self.negative_rotation=False
        
    def callback_set_gripper_angle(self,msg):
        print("setting gripper to angle:",msg.data,type(msg.data))
        self.servo_roll.angle=msg.data
    
    def timer_callback(self):
        try:
            msg = String()
            self.value=(21000-self.chan.value)/1000
            msg.data = f'{self.value}'
            self.publisher.publish(msg)
            #self.get_logger().info(f'Publishing: "{msg.data}"')
            
            if self.close_gripper:
                if self.value>1.5 and self.angle>120:
                    if self.rotation_trigger and not self.rotation:
                        self.rotation=True
                        self.rotation_trigger=False
                        print('here')
                    self.angle-=1
                    self.servo1.angle = self.angle
                    self.servo2.angle = self.angle
                    self.servo3.angle = self.angle
                if self.value<1 and self.angle<160:
                    self.angle+=1
                    self.servo1.angle = self.angle
                    self.servo2.angle = self.angle
                    self.servo3.angle = self.angle
                now = self.get_clock().now()
                print(self.value,self.angle,now - self.close_gripper_start_time)
                if now - self.close_gripper_start_time > Duration(seconds=self.wait_time):
                    self.get_logger().info('30 seconds passed. Reopening gripper.')
                    self.angle=30
                    self.servo1.angle = self.angle
                    self.servo2.angle = self.angle
                    self.servo3.angle = self.angle
                    self.close_gripper = False

            if self.rotation:
                print(self.yaw_angle)
                if not self.negative_rotation and not self.postive_rotation:
                    self.yaw_angle-=1
                    if self.yaw_angle<30:
                        self.negative_rotation=True
                if self.negative_rotation:
                    self.yaw_angle+=1
                    if self.yaw_angle>150:
                        self.negative_rotation=False
                        self.postive_rotation=True
                if self.postive_rotation:
                    self.yaw_angle-=1
                    print("here")
                    if self.yaw_angle<=90:
                        self.rotation=False
                        self.postive_rotation=False
                self.servo_yaw.angle=self.yaw_angle
        except Exception as e:
            print(e)
            
    
    def close_gripper_service_callback(self, request, response):
        self.get_logger().info("closing gripper called!")
        self.close_gripper=True
        self.rotation_trigger=True
        self.close_gripper_start_time = self.get_clock().now()  
        response.success = True  # Indicate success
        response.message = "Trigger received and processed"
        return response
        
    def open_gripper_service_callback(self, request, response):
        self.get_logger().info("open gripper called!")
        self.close_gripper=False
        self.angle=30
        self.servo1.angle = self.angle
        self.servo2.angle = self.angle
        self.servo3.angle = self.angle
        response.success = True  # Indicate success
        response.message = "Trigger received and processed"
        return response
def main(args=None):
    rclpy.init(args=args)  # Initialize the ROS client library
    node = TimerNode()     # Create an instance of the node

    try:
        rclpy.spin(node)   # Keep the node running and process callbacks
    except KeyboardInterrupt:
        pass
    finally:
        # Set gripper to 0 position before shutting down
        node.get_logger().info("Shutting down. Resetting gripper to 0 position.")
        node.servo1.angle = 30
        node.servo2.angle = 30
        node.servo3.angle = 30
        node.servo_yaw.angle = 90
        node.servo_roll.angle = 90
        time.sleep(1)  # Optional: give time for servo to reach position

        node.destroy_node()  # Clean up and destroy the node
        rclpy.shutdown()  # Shutdown ROS 2 client library

if __name__ == '__main__':
    main()

