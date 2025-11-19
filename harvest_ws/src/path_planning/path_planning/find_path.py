import rclpy
from rclpy.node import Node
from .RRT.RRT import IK_MDN_Model,MDNLayer,RRTAngle,forward_kinematics_from_angles
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import pickle

from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped, Pose
from nav_msgs.msg import Path

import torch.nn as nn
import torch
import torch.nn.functional as F
import numpy as np
import time
import redis


broker="172.23.66.117"
redis_server='172.23.248.33'

pool = redis.ConnectionPool(host=redis_server, port=6379, decode_responses=True, password='jimmy')
r = redis.Redis(connection_pool=pool)

def find_path(goal,start=[0, 0,0],obstacle=[]):
    all_path=[]
    angle_rrt = RRTAngle(start,tolerance=0.1, step_size=3.1415/180*5)
    for x,y,w,h in obstacle:
        angle_rrt.add_obstacle('rectangle', (x,y,w,h))
    path1=angle_rrt.build_tree(goal,goal_bias=0.5,max_iterations=1000)

    if path1 is not None:

        all_path.extend(path1)
        angle_rrt = RRTAngle(path1[-1],tolerance=0.05, step_size=3.1415/180*2)
        for x,y,w,h in obstacle:
            angle_rrt.add_obstacle('rectangle', (x,y,w,h))
        path2=angle_rrt.build_tree(goal,goal_bias=0.5,max_iterations=1000)
    else:
        angle_rrt = RRTAngle(start,tolerance=0.1, step_size=3.1415/180*5)
        for x,y,w,h in obstacle:
            angle_rrt.add_obstacle('rectangle', (x,y,w,h))
        path2=angle_rrt.build_tree(goal,goal_bias=0.5,max_iterations=1000)
    
    
    if path2 is not None:
        all_path.extend(path2)
        angle_rrt = RRTAngle(path2[-1],tolerance=0.01, step_size=3.1415/180*2)
        for x,y,w,h in obstacle:
            angle_rrt.add_obstacle('rectangle', (x,y,w,h))
        path3=angle_rrt.build_tree(goal,goal_bias=1)
    else:
        path3=None

    if path3 is not None:
        all_path.extend(path3)

    return all_path


# class PathPlanningNode(Node):

#     def __init__(self):
#         super().__init__('path_planning')
              
#         self.obstcles=np.load("/home/cotrobot/laser_scan/obstacles.npy")
#         self.PrismaticJointZ,self.theta1,self.theta2,self.theta3,self.gripperJoint1,self.gripperJoint2,selfgripperJoint3=0,0,0,0,0,0,0
#         self.joint_name=["PrismaticJointZ","Angle1","Angle2","Angle3","GripperJoint1","GripperJoint2","GripperJoint3"]
        
#         self.joint_state_suscriber = self.create_subscription(JointState,"/joint_states",self.update_joint_states,10)

#         self.sub = self.create_subscription(PoseStamped,'/goal_pose', self.goal_callback,10)
        
#     def goal_callback(self, msg):
#         self.get_logger().info(
#             f"Received goal pose:\n"
#             f"  Position: x={msg.pose.position.x}, y={msg.pose.position.y}\n"
#             f"  Orientation: z={msg.pose.orientation.z}, w={msg.pose.orientation.w}"
#         )
#         path=(find_path([msg.pose.position.x,msg.pose.position.y],[self.theta1,self.theta2,self.theta3],self.obstcles))
#         points_path=[forward_kinematics_from_angles(*angles)[-1] for angles in path]

#     def update_joint_states(self,msg):
#         self.theta1,self.theta2,self.theta3=msg.position[1],msg.position[2],msg.position[3]    


# def main(args=None):
    
    
#     rclpy.init(args=args)

#     minimal_publisher = PathPlanningNode()

#     rclpy.spin(minimal_publisher)
#     minimal_publisher.destroy_node()
#     rclpy.shutdown()


class PathPlanningNode(Node):

    def __init__(self):
        super().__init__('path_planning')
              
        self.obstacles = np.load("/home/cotrobot/laser_scan/obstacles.npy")
        self.theta1, self.theta2, self.theta3 = 0, 0, 0
        self.joint_name = ["PrismaticJointZ", "Angle1", "Angle2", "Angle3", "GripperJoint1", "GripperJoint2", "GripperJoint3"]
        
        # Timer for publishing joint commands
        self.timer_ = self.create_timer(0.1, self.timer_callback)

        # Subscribers
        self.joint_state_subscriber = self.create_subscription(JointState, "/joint_states", self.update_joint_states, 10)
        self.goal_subscriber = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)

        # Publisher for planned path (for RViz)
        self.path_publisher = self.create_publisher(Path, '/planned_path', 10)
        # Publisher for joint states
        self.joint_state_publisher = self.create_publisher(JointState, "/joint_states", 10)

        self.path = []  # stores planned joint path
        self.robot_free=True

        self.robot_move_status_sub = self.create_subscription(String, "move_status",self.update_move_status, 10)
    
    def update_move_status(self,msg):
        if msg.data=="Moving":
            self.robot_free=False
        else:
            self.robot_free=True



    def timer_callback(self):
        if len(self.path) > 0 and self.robot_free:
            next_joint_angles = self.path.pop(0)
            
            # Create JointState message
            joint_msg = JointState()
            joint_msg.header.stamp = self.get_clock().now().to_msg()
            joint_msg.name = self.joint_name
            # Assuming PrismaticJointZ and the first three angles are relevant
            joint_msg.position = [0.0, next_joint_angles[0], next_joint_angles[1], next_joint_angles[2], 0.0, 0.0, 0.0]

            self.joint_state_publisher.publish(joint_msg)
            self.get_logger().info(f"Published joint command: {joint_msg.position}")

    def goal_callback(self, msg):
        r.set("mode","camera_done")
        self.get_logger().info(
            f"Received goal pose:\n"
            f"  Position: x={msg.pose.position.x}, y={msg.pose.position.y}\n"
            f"  Orientation: z={msg.pose.orientation.z}, w={msg.pose.orientation.w}"
        )

        # Find path in joint space (list of joint angles)
        self.path = find_path([msg.pose.position.x, msg.pose.position.y],
                         [self.theta1, self.theta2, self.theta3],
                         self.obstacles)
        
        # Convert joint path to end-effector poses
        points_path = [forward_kinematics_from_angles(*angles)[-1] for angles in self.path]
        print(points_path)
        # Create PoseArray to publish
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "base_link"

        for point in points_path:
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.header.frame_id = "base_link"
            pose_stamped.pose.position.x = point[0]
            pose_stamped.pose.position.y = point[1]
            pose_stamped.pose.position.z = 0.0
            pose_stamped.pose.orientation.w = 1.0  # default orientation
            path_msg.poses.append(pose_stamped)

        
        self.path_publisher.publish(path_msg)
        self.get_logger().info(f"Published path with {len(points_path)} points.")
        r.set("mode","camera_ready")

    def update_joint_states(self, msg):
        self.theta1, self.theta2, self.theta3 = msg.position[1], msg.position[2], msg.position[3]

def main(args=None):
    rclpy.init(args=args)
    path_planner = PathPlanningNode()
    rclpy.spin(path_planner)
    path_planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
