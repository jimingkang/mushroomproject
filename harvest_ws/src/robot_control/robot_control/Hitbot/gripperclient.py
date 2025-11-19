import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import numpy as np
from std_msgs.msg import String,Int32
from scipy.spatial.transform import Rotation as R
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped, Pose
from nav_msgs.msg import Path
from std_srvs.srv import Trigger
class ServiceClient(Node):
    def __init__(self):
        super().__init__('service_client')
        self.close_client = self.create_client(Trigger, 'close_gripper_service')  # Service type and name
        self.open_client = self.create_client(Trigger, 'open_gripper_service')  # Service type and name
       
        # Wait for service to be available
        while not self.close_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('close service not available, waiting again...')
        while not self.open_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('open service not available, waiting again...')
       
        self.close_request = Trigger.Request()
        self.open_request = Trigger.Request()
   
    def close_send_request(self ):
        #self.request.a = a
        #self.request.b = b
        self.future = self.close_client.call_async(self.close_request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    def open_send_request(self ):
        #self.request.a = a
        #self.request.b = b
        self.future = self.open_client.call_async(self.open_request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()