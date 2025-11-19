import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from visualization_msgs.msg import Marker
import numpy as np

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        
        self.publisher_ = self.create_publisher(Marker, 'obs_marker', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.obstcles=np.load("/home/cotrobot/laser_scan/obstacles.npy")
        self.theta1,self.theta2,self.theta3=0,0,0
        self.joint_state_suscriber = self.create_subscription(JointState,"/joint_states",self.update_joint_states,10)
        
    def timer_callback(self):
        
        for id,obstcale in enumerate(self.obstcles):
            x1,y1,w,h=obstcale
            marker = Marker()
            marker.header.frame_id = "base_link"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = f"obs{id+1}"
            marker.id = 0
            marker.type = Marker.CUBE
            marker.action = Marker.ADD

            marker.pose.position.x = x1+w/2
            marker.pose.position.y = y1+h/2
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0

            marker.scale.x = w
            marker.scale.y = h
            marker.scale.z = 1.0
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.color.a = 1.0
            self.publisher_.publish(marker)
     
    def update_joint_states(self,msg):
        self.theta1,self.theta2,self.theta3=msg.position[1],msg.position[2],msg.position[3]    


def main(args=None):
    
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()