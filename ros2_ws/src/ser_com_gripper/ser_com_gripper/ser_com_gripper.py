import serial
import rclpy
import time
import signal
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

def map_value(value, old_min=90, old_max=157, new_min=-0.55, new_max=-1.7):
    return new_min + (value - old_min) * (new_max - new_min) / (old_max - old_min)


class ArduinoMessage:
    def __init__(self, message):
        if message is None:
            message = ''  # If the message is None, assign an empty string
        message_list = message[:-1].split(',')  # Remove the trailing ';' and split by commas
        
        # Initialize the attributes with default values
        self.time = None
        self.sensor_value = None
        self.angle1 = None
        self.angle2 = None
        self.angle3 = None
        self.stop = None
        
        
        # Iterate over each key-value pair in the message
        for single_message in message_list:
            if 'time: ' in single_message:
                self.time = int(single_message.split(': ')[1])  # Extract and convert 'time' to an integer
            elif 'sensor_value: ' in single_message:
                self.sensor_value = int(single_message.split(': ')[1])  # Extract and convert 'sensor_value'
            elif 'angle1: ' in single_message:
                self.angle1 = int(single_message.split(': ')[1])  # Extract and convert 'angle1'
            elif 'angle2: ' in single_message:
                self.angle2 = int(single_message.split(': ')[1])  # Extract and convert 'angle2'
            elif 'angle3: ' in single_message:
                self.angle3 = int(single_message.split(': ')[1])  # Extract and convert 'angle3'
            elif 'stop: ' in single_message:
                self.stop = single_message.split(': ')[1]  # Extract the 'stop' value (as string)

    def __repr__(self):
        return f"ArduinoMessage(time={self.time}, sensor_value={self.sensor_value}, " \
               f"angle1={self.angle1}, angle2={self.angle2}, angle3={self.angle3}, stop={self.stop})"

class GripperNode(Node):
    def __init__(self, port='/dev/ttyACM0'):
        super().__init__('gripper_node')
        self.ser = None
        print('Initializing Gripper Node...')
        if self.ser is None:
            try:
                self.ser = serial.Serial(port)
                self.get_logger().info(f'Connection to {port} successful.')
                self.ser.reset_input_buffer()
                self.ser.reset_output_buffer()
                self.ser.flush()
                self.timer = self.create_timer(1.0, self.timer_callback)
                time.sleep(0.25)
            
            except Exception as e:
                self.get_logger().error(f'Connection to {port} failed: {e}')
            #self.close_serial_connection()
        self.residual_message=''
        self.last_message=ArduinoMessage(self.residual_message)
        
        
        self.srv_close = self.create_service(Trigger, 'close_gripper', self.close_gripper_callback)
        self.srv_open = self.create_service(Trigger, 'open_gripper', self.open_gripper_callback)
        self.joint_publisher = self.create_publisher(JointState, 'joint_states', 10)

    
    def timer_callback(self):
        if self.ser is None:
            return
        self.get_logger().info('get_Data_from_arduino')
        s = self.ser.read_all()
        try:
            self.residual_message=self.residual_message+ s.decode()
        except Exception as e:
            self.get_logger().error(f'failed decoding message {e}')
            
        lines=self.residual_message.split(';\r\n') 
        message=None
        if len(lines)>2:

            for i in range(len(lines)-1,0,-1):
                line=lines[i]
                if len(line)<2:
                    continue
                if line[-1]!=';':
                    
                    self.residual_message=line
                else:
                    print(line)
                    message=line
                    self.residual_message=''
                    break
        else:
            self.get_logger().warn('empty message recived')
        if message is None:
            self.get_logger().warn('Message parsing failed')
        self.last_message=ArduinoMessage(message)
        
        joint_state=JointState()
        if self.last_message.angle1 is not None:
            joint_state.header.stamp = self.get_clock().now().to_msg()
            joint_state.name=['Joint1','Joint2','JointG1','JointG2','JointG3']
            joint_state.position=[0.0, 0.0, map_value(self.last_message.angle1),map_value(self.last_message.angle2),map_value(self.last_message.angle3)]
            self.joint_publisher.publish(joint_state)

        self.get_logger().info(f'{self.last_message}')
        time.sleep(0.5)

    def close_gripper_callback(self, request, response):
        self.ser.reset_output_buffer()
        time.sleep(0.25)
        self.ser.write(b'go')
        time.sleep(0.25)
        response.success=True
        return response
    
    def open_gripper_callback(self, request, response):
        self.ser.reset_output_buffer()
        time.sleep(0.25)
        self.ser.write(b'.')
        time.sleep(0.25)
        response.success=True
        return response

    def close_serial_connection(self):
        self.get_logger().info('Serial connection closed.')
        if self.ser is not None:
            self.ser.close()
            

def signal_handler(signal, frame):
    """Handle the SIGINT signal (Ctrl+C)."""
    print('Ctrl+C pressed, shutting down gracefully...')
    global gripper
    if gripper:
        gripper.close_serial_connection()
    rclpy.shutdown()
    exit(0)  # Exit the program


def main():
    rclpy.init()
    global gripper
    gripper = GripperNode(port='/dev/ttyACM0')
    
    try:
        while rclpy.ok():
            rclpy.spin_once(gripper)  
            signal.signal(signal.SIGINT, signal_handler)
            time.sleep(0.1) 

    except KeyboardInterrupt:
        print('Node shutdown requested.')
        print('here')
        if gripper:
            gripper.close_serial_connection()
    
    

if __name__ == '__main__':
    main()
