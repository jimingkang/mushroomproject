
import numpy as np
import os

from ikpy.chain import Chain
from ikpy.link import URDFLink
import rclpy

from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import Point 

def chain_bulider(link_lengths= [np.float64(0.325), np.float64(0.275), np.float64(0.26)]):
    scara_chain = Chain(name='RRR_SCARA', links=[
        URDFLink(
            name="base",
            origin_translation=[0, 0, 0],
            origin_orientation=[0, 0, 0],  # roll, pitch, yaw
            rotation=[0, 0, 1],  # rotate around Z
            joint_type='revolute'
        ),
        URDFLink(
            name="link1",
            origin_translation=[link_lengths[0], 0, 0],
            origin_orientation=[0, 0, 0],
            rotation=[0, 0, 1],
            joint_type='revolute'
        ),
        URDFLink(
            name="link2",
            origin_translation=[link_lengths[1], 0, 0],
            origin_orientation=[0, 0, 0],
            rotation=[0, 0, 1],
            joint_type='revolute'
        ),
        URDFLink(
            name="end_effector",
            origin_translation=[link_lengths[2], 0, 0],
            origin_orientation=[0, 0, 0],
            rotation=None,  # fixed segment
            joint_type='fixed'
        )
    ])

    return scara_chain

def forward_kinematics_from_angles(theta,link_lengths):
    theta1, theta2, theta3 = theta
    x =  link_lengths[0]* np.cos(theta1) + link_lengths[1] * np.cos(theta1 + theta2) + link_lengths[2] * np.cos(theta1 + theta2 + theta3)
    y = link_lengths[0] * np.sin(theta1) + link_lengths[1] * np.sin(theta1 + theta2) + link_lengths[2] * np.sin(theta1 + theta2 + theta3)
    return np.array([x, y])

class IKCalcNode(Node):
    def __init__(self, xml_path):
        
        self.link_lengths=[np.float64(0.325), np.float64(0.275), np.float64(0.26)]

        super().__init__('ik_calc_node')
        self.robot=chain_bulider(self.link_lengths)
        
        #self.target_position_subscriber = self.create_subscription(Point,'target_position',self.ik_calc_callback_fall_back, 10)
        self.publisher_angle1 = self.create_publisher(Int32,'angle_theta1', 10)
        self.publisher_angle2 = self.create_publisher(Int32,'angle_theta2', 10)
        self.publisher_angle3 = self.create_publisher(Int32,'angle_theta3', 10)
        
    def forward_kinematics_from_angles(self,theta):
        theta1, theta2, theta3 = theta
        x = self.link_lengths[0] * np.cos(theta1) + self.link_lengths[1] * np.cos(theta1 + theta2) + self.link_lengths[2] * np.cos(theta1 + theta2 + theta3)
        y = self.link_lengths[0] * np.sin(theta1) + self.link_lengths[1] * np.sin(theta1 + theta2) + self.link_lengths[2] * np.sin(theta1 + theta2 + theta3)
        return np.array([x, y])

    def ik_calc_callback(self,msg):
        
        joint_angles = self.robot.inverse_kinematics([msg.x,msg.y,0])
        self.angle1,self.angle2,self.angle3=joint_angles[0],joint_angles[1],joint_angles[2]
        
        joint_angles_degree = [round(i*180/3.1415) for i in joint_angles[:3]]
        theta1,theta2,theta3 = Int32(),Int32(),Int32()
        theta1.data=joint_angles_degree[0]
        theta2.data=joint_angles_degree[1]
        theta3.data=joint_angles_degree[2]
        self.publisher_angle1.publish(theta1)
        
        self.publisher_angle2.publish(theta2)
        
        self.publisher_angle3.publish(theta3)


def main(args=None):
    xml_path = '/home/a/Downloads/mushroomproject/ros2_ws/build/hitbot_sim/hitbot_sim/scara_ik.xml'  # XML file

    rclpy.init(args=args)
    node = IKCalcNode(xml_path)

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



 