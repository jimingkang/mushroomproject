import io
import math
import pickle
import subprocess
import sys
import os
import rclpy
import time
from rclpy.node import Node
#from .policy import ACTPolicy
from std_msgs.msg import Int64,String
from sensor_msgs.msg import JointState,Image
from hitbot_msgs.srv import *
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Float64MultiArray,Int32
from moveit_msgs.msg import DisplayTrajectory
import redis
from octomap_msgs.msg import Octomap
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
import numpy as np
import random
import octomap
from scipy.spatial import KDTree
import fcl
from ikpy.chain import Chain
from ikpy.link import URDFLink
import h5py
from collections import deque
from cv_bridge import CvBridge
import cv2
import pygame
#import torch
import threading
# Load the URDF file and create the chain
from example_interfaces.srv import Trigger
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

#redis_server='10.0.0.21'
redis_server='172.23.248.34'

pool = redis.ConnectionPool(host=redis_server, port=6379, decode_responses=True,password='jimmy')
r = redis.Redis(connection_pool=pool)


os.chdir(os.path.expanduser('~'))
#sys.path.append("./ws_moveit/src/hitbot")  ## get import pass: hitbot_interface.py
#from .hitbot_interface import HitbotInterface
from .HitbotInterface import HitbotInterface


import numpy as np
from scipy.optimize import root
from matplotlib import pyplot as plt
from .transform_calc import RobotTransform

class Solver:
    def __init__(self,link_lengths=np.array([0.325, 0.275, 0.260])):
        self.link_lengths = link_lengths

    def forward_kinematics_from_angles(self,theta):
        theta1, theta2, theta3 = theta
        x = self.link_lengths[0] * np.cos(theta1) + self.link_lengths[1] * np.cos(theta1 + theta2) + self.link_lengths[2] * np.cos(theta1 + theta2 + theta3)
        y = self.link_lengths[0] * np.sin(theta1) + self.link_lengths[1] * np.sin(theta1 + theta2) + self.link_lengths[2] * np.sin(theta1 + theta2 + theta3)
        return np.array([x, y])

    def forward_kinematics_from_angles_all(self,theta):
        theta1, theta2, theta3 = theta
        x1 = self.link_lengths[0] * np.cos(theta1)
        y1 = self.link_lengths[0] * np.sin(theta1)
        x2 = x1 + self.link_lengths[1] * np.cos(theta1 + theta2)
        y2 = y1 + self.link_lengths[1] * np.sin(theta1 + theta2)
        x3 = x2 + self.link_lengths[2] * np.cos(theta1 + theta2 + theta3)
        y3 = y2 + self.link_lengths[2] * np.sin(theta1 + theta2 + theta3)
        return np.array([[x1, y1], [x2, y2], [x3, y3]])

    def error_function(self,theta):
        x,y=self.target-self.forward_kinematics_from_angles(theta)
        dx=x*0.001
        dy=y*0.001
        return np.array([self.compute_dtheta1(dx, dy, theta[0], theta[1], theta[2]),self.compute_dtheta2(dx, dy, theta[0], theta[1], theta[2]),self.compute_dtheta3(dx, dy, theta[0], theta[1], theta[2])])

    def compute_dtheta1(self,dx, dy, theta1, theta2, theta3):
        r1,r2,r3=self.link_lengths
        jx = -r1 * np.sin(theta1) - r2 * np.sin(theta1 + theta2) - r3 * np.sin(theta1 + theta2 + theta3)
        jy =  r1 * np.cos(theta1) + r2 * np.cos(theta1 + theta2) + r3 * np.cos(theta1 + theta2 + theta3)
        J_theta1 = np.array([jx, jy])
        delta_p = np.array([dx, dy])
        dtheta1 = np.dot(J_theta1, delta_p) / np.dot(J_theta1, J_theta1)
        return dtheta1

    def compute_dtheta2(self,dx, dy, theta1, theta2, theta3):
        r1,r2,r3=self.link_lengths
        jx = - r2 * np.sin(theta1 + theta2) - r3 * np.sin(theta1 + theta2 + theta3)
        jy = r2 * np.cos(theta1 + theta2) + r3 * np.cos(theta1 + theta2 + theta3)
        J_theta1 = np.array([jx, jy])
        delta_p = np.array([dx, dy])
        dtheta1 = np.dot(J_theta1, delta_p) / np.dot(J_theta1, J_theta1)
        return dtheta1

    def compute_dtheta3(self,dx, dy, theta1, theta2, theta3):
        r1,r2,r3=self.link_lengths
        jx = - r3 * np.sin(theta1 + theta2 + theta3)
        jy = r3 * np.cos(theta1 + theta2 + theta3)
        J_theta1 = np.array([jx, jy])
        delta_p = np.array([dx, dy])
        dtheta1 = np.dot(J_theta1, delta_p) / np.dot(J_theta1, J_theta1)
        return dtheta1

    def cost(self, ik_success, current_angle):
        ik_success = np.array(ik_success)
        current_value = self.forward_kinematics_from_angles_all(current_angle)
        angle_costs=[]
        # weights = np.array([[2.0,2.0],[ 1.0,1.0], [1,1]])
        # weights2 = np.array([[1.5,1.5],[ 1.0,1.0], [1,1]])

        for i in ik_success:
            #values=self.forward_kinematics_from_angles_all(i)
            val1=abs(i[0]-current_angle[0])*100 if i[0]>0 else abs(i[0]-current_angle[0])*80
            val2=abs(i[1]-current_angle[1]) *1000
            val3=abs(i[2]-current_angle[2])*8 if i[2]>0  else abs(i[2]-current_angle[2])*10
            angle_costs.append(np.linalg.norm(val1+val2+val3))

        return np.array(angle_costs)

    def solve(self,target,current_angle):
        self.target= np.array(target)
        limits=[[-np.pi/2,np.pi/2],[-np.pi/1.3,np.pi/1.3],[-np.pi/2,np.pi/2]]
        initial_guesses=[[np.random.uniform(*limits[i]) for i in range(3)] for i in range(0,100)]
        ik_success=[]
        for guess in initial_guesses:
            sol=root(self.error_function, guess, method='hybr')
            if sol.success:
                error = np.linalg.norm(self.target-self.forward_kinematics_from_angles(sol.x))
                x = sol.x

                within_limits = all(
                    limits[i][0] <= x[i] <= limits[i][1]
                    for i in range(3)
                )

                if error < 0.01 and within_limits:
                    ik_success.append(x)
        if len(ik_success):
            ik_success=np.array(ik_success)
            angle_cost=self.cost(ik_success, current_angle)
            return ik_success,angle_cost
        else:
            return target,current_angle

    def get_angle_to_target(self, target, current_angle):
        """
        Computes the optimal joint configuration (angle) required to reach a given target from the current angle.

        This function uses the `solve` method to calculate possible solutions and their associated angle costs,
        then selects and returns the configuration with the minimum cost.

        Parameters:
        ----------
        target : array-like
            The desired position or state that the system should reach.
        current_angle : array-like or float
            The current angle or joint configuration of the system.

        Returns:
        -------
        array-like or float
            The angle configuration from the list of solutions that has the minimum associated cost.
        """
        solved, angle_cost = self.solve(target, current_angle)
        min_config = np.argmin(angle_cost)
        try:
            return solved[min_config]
        except IndexError:
            print(f"Error: No solution at index {min_config} (total solutions: {len(solved)})")
            return None  # or raise a more descriptive exception

        #return solved[min_config]
    def get_robot_angle_in_degree(self, target, current_angle=[0,0,0]):
        ans=self.get_angle_to_target(target, current_angle)
        if ans is None:  # Check if IK failed
            print(f"ERROR: No valid joint angles (IK failed)! current_angle={current_angle}")
            ans= current_angle #[0.0, 0.0, 0.0]  # Default safe value (or raise an exception)
        return [round(ans[0]*180/3.1415,0),round(ans[1]*180/3.1415,0),round((ans[0]+ans[1]+ans[2])*180/3.1415,0)]


class ServiceClient(Node):
    def __init__(self):
        super().__init__('service_client')
        self.client = self.create_client(Trigger, 'close_gripper_service')  # Service type and name
        self.open_client = self.create_client(Trigger, 'open_gripper_service')  # Service type and name
       
        # Wait for service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('close service not available, waiting again...')
        while not self.open_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('open service not available, waiting again...')
       
        self.request = Trigger.Request()
        self.open_request = Trigger.Request()
   
    def send_request(self ):
        #self.request.a = a
        #self.request.b = b
        self.future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    def open_send_request(self ):
        #self.request.a = a
        #self.request.b = b
        self.future = self.open_client.call_async(self.open_request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


class HitbotController(Node):
    def __init__(self):
        super().__init__('hitbot_controller')
        # 使用 ReentrantCallbackGroup 允许并发执行
        #self.group1 = ReentrantCallbackGroup()
        #self.group2 = ReentrantCallbackGroup()
        r.set("mode","camera_ready")

        self.client_node = ServiceClient()
        self.solver=Solver()
        self.goal=[0,0]

        self.link_lengths=np.array([0.325, 0.275, 0.260])
        self.rt=RobotTransform(self.link_lengths)
        self.hitbot_x = 0
        self.hitbot_y = 0
        self.hitbot_z = 0
        self.hitbot_r = -180

        self.hitbot_t1_publisher = self.create_subscription(Int32, '/hitbot_theta1',self.hitbot_theta1_callback, 10)
        self.hitbot_t2_publisher = self.create_subscription(Int32, '/hitbot_theta2',self.hitbot_theta2_callback, 10)
        self.hitbot_t2_publisher = self.create_subscription(Int32, '/hitbot_theta3',self.hitbot_theta3_callback,10)
        self.hitbot_t3_publisher = self.create_subscription(Int32, '/hitbot_move_z', self.hitbot_move_z_callback,10)

        self.hitbot_x_publisher = self.create_publisher(String, '/hitbot_x', 10)
        self.hitbot_y_publisher = self.create_publisher(String, '/hitbot_y', 10)
        self.hitbot_z_publisher = self.create_publisher(String, '/hitbot_z', 10)
        self.hitbot_r_publisher = self.create_publisher(String, '/hitbot_r', 10)
        self.camera_xyz_publisher = self.create_publisher(String, '/camera_xyz', 10)


        self.bounding_boxes_sub = self.create_subscription(String,"/yolox/bounding_boxes",self.bounding_boxes_callback, 10)
        self.adj_bounding_boxes_sub = self.create_subscription(String,"/d405/yolox/adj_bounding_boxes",self.adj_bounding_boxes_callback, 10)
        self.xyz_sub = self.create_subscription(String,"/hitbot_end_xyz",self.hitbot_end_xyzr_callback,10)
        self.angle_sub = self.create_subscription(String,"/hitbot_end_angle",self.hitbot_end_angle_callback,10)
        self.gripper_adjust_sub = self.create_subscription(String,"/yolox/rpi5/adjust/xy_pixel",self.hitbot_gripper_adjust_callback,1)
        self.gripper_adj_done_sub = self.create_subscription(String,"/yolox/rpi5/adjust/done",self.hitbot_gripper_adjust_done_callback,1)
        self.gripper_adj_done_pub= self.create_publisher(String,'/yolox/rpi5/adjust/done',1)
        self.rpi5_adj_xy_pixel=[0,0]
        
        

        self.gripper_open_pub= self.create_publisher(String,'/yolox/gripper_open',10)
        self.gripper_hold_pub = self.create_publisher(String,'/yolox/gripper_hold',10)
        
        # Joint State Publisher
        self.joint_state_pub = self.create_publisher(JointState, "/hitbot/joint_states", 10)

        # Timer to publish joint states at 50Hz (20ms)
        self.timer = self.create_timer(0.5, self.publish_joint_states)

        # Define joint names (Modify according to your HitBot model)
        self.joint_names = ["joint1", "joint2", "joint3", "joint4"]
       

        self.joint_command_sub = self.create_subscription(
            DisplayTrajectory,
            "/display_planned_path",
            #"/scara_arm_controller/joint_trajectory",
            self.joint_command_callback,
            10
        )
        # Publisher to send commands to HitBot controller
        self.hitbot_command_pub = self.create_publisher(
            Float64MultiArray,
            "/hitbot/set_joint_positions",
            10
        )


        self.robot_id = 92  ## 123 is robot_id, Modify it to your own
        self.robot = HitbotInterface(self.robot_id)

        self.init_robot()
        #pygame.init()
        #pygame.display.set_mode((100, 100))  # Small invisible window
        #pygame.display.set_caption("ROS2 Keyboard Control")

        #self.urdf_file = "/home/a/Downloads/mushroomproject/ros2_ws/build/hitbot_sim/hitbot_sim/scara_ik.xml"
        #self.scara_arm = Chain.from_urdf_file(self.urdf_file)
        self.R = np.array([
        [0, -1, 0], #[1,0,0]
        [0, 0, -1],  # [0, 1, 0],
        [1, 0, 0]    #        [0,0,1]
        ])

        #self.subscription = self.create_subscription(Octomap,'/rtabmap/octomap_binary',self.octomap_callback,10)

        # Publisher for RRT Path Visualization
        self.path_publisher = self.create_publisher(Marker, 'rrt_path', 10)
        self.fcl_octree = None  # FCL Octree for collision checking
        self.occupancy_map = None  # Store Octomap for collision checking
        self.bounds = (-2.0, 2.0, -2.0, 2.0, -2.0, 2.0)  # Workspace bounds (xmin, xmax, ymin, ymax, zmin, zmax)


        #self.dataset_dir = "dataset_dir"
        #os.makedirs(self.dataset_dir, exist_ok=True)
        
        # SCARA configuration
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
        self.gripper_joint = 'joint4'  # Assuming last joint controls gripper
        
        # Data buffers
        self.buffer = {
            'observations': {
                'images': deque(maxlen=1000),
                'qpos': deque(maxlen=1000),
                'cartesian_pos': deque(maxlen=1000),
                'gripper': deque(maxlen=1000)
            },
            'action': deque(maxlen=1000),
            'timestamps': deque(maxlen=1000)
        }
        
        # ROS setup
        self.bridge = CvBridge()
        #self.joints_sub=self.create_subscription( JointState,"/hitbot/joint_states", self.joint_state_cb,10)
        #self.image_sub=self.create_subscription(Image,"/camera/color/image_rect_raw",  self.image_cb,10)  #/yolox/boxes_image
        
        self.recording = False
        self.episode_count = 0
        self.last_qpos = None
        self.last_cartesian_pos=None

        self.latest_image=None
        self.latest_qpos=None
        self.link_lengths=[np.float64(0.325), np.float64(0.275), np.float64(0.26)]
        self.ik_solver=self.chain_bulider(self.link_lengths)
    
        #self.mean = torch.tensor([0.485, 0.456, 0.406]).cuda().view(3, 1, 1)
        #self.std = torch.tensor([0.229, 0.224, 0.225]).cuda().view(3, 1, 1)
    def chain_bulider(self,link_lengths= [np.float64(0.325), np.float64(0.275), np.float64(0.26)]):
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
    def hitbot_theta1_callback(self,msg):
        theta1=msg.data
        theta1=int(theta1)
        self.get_logger().info(f"get theta1:{theta1}")
        self.robot.movej_angle(theta1,self.robot.angle2,self.robot.z,self.robot.r,50,1)
    def hitbot_theta2_callback(self,msg):
        theta2=msg.data
        theta2=int(theta2)
        self.get_logger().info(f"get theta2:{theta2}")
        self.robot.movej_angle(self.robot.angle1,theta2,self.robot.z,self.robot.r,50,1)
    def hitbot_theta3_callback(self,msg):
        theta3=msg.data
        theta3=int(theta3)
        self.get_logger().info(f"get theta3:{theta3}")
        self.robot.movej_angle(self.robot.angle1,self.robot.angle2,self.robot.z,theta3,50,1)
    def forward_kinematics_from_angles(self,theta):
        theta1, theta2, theta3 = theta[:3]
        x = self.link_lengths[0] * np.cos(theta1) + self.link_lengths[1] * np.cos(theta1 + theta2) + self.link_lengths[2] * np.cos(theta1 + theta2 + theta3)
        y = self.link_lengths[0] * np.sin(theta1) + self.link_lengths[1] * np.sin(theta1 + theta2) + self.link_lengths[2] * np.sin(theta1 + theta2 + theta3)
        #x = self.link_lengths[0] * np.cos(theta1) + self.link_lengths[1] * np.cos(theta1 + theta2) + self.link_lengths[2] * np.cos(theta1 + theta2 + theta3)
        #y = self.link_lengths[0] * np.sin(theta1) + self.link_lengths[1] * np.sin(theta1 + theta2) + self.link_lengths[2] * np.sin(theta1 + theta2 + theta3)
        return np.array([x, y])
    
    def bounding_boxes_callback(self, msg):
        if r.get("mode")=="camera_ready":
            mushroom_xyz=msg.data
            mushroom_xyz=msg.data.split(",");
            goal=[int(float(mushroom_xyz[2].strip()))-270,0-int(float(mushroom_xyz[0].strip()))]
            #self.get_logger().info(f"get mushroom_xyz:{mushroom_xyz}")
            #self.goal=[int(float(mushroom_xyz[2].strip()))/1000,0-int(float(mushroom_xyz[0].strip()))/1000]
            #self.get_logger().info(f"target get goal:{self.goal}")
            current_angle=[self.robot.angle1 *3.1415/180,self.robot.angle2*3.1415/180,self.robot.r*3.1415/180]
            #angles=self.solver.get_robot_angle_in_degree(self.goal,current_angle)
            #self.get_logger().info(f"Computed   angle:{angles},current_angle:{current_angle}")
            #computed_pos=self.forward_kinematics_from_angles(angles[:3])
            #self.get_logger().info(f"Computed   position:{computed_pos}")
            #if (abs(current_angle[2]))*180/3.14>100:
                #r.set("mode","camera_ready")
            #    self.get_logger().info(f"self collide")
                #return

            #ret=self.robot.movej_angle(angles[0],angles[1],0,angles[2]-180,100,1) 
            ret=self.robot.movej_xyz(goal[0],goal[1],0,-180,80,1)
            self.get_logger().info(f"bounding_boxes_callback : goal={goal},ret :{ret}")
            self.robot.wait_stop()
            if ret<2:
                r.set("mode","adjust_ready")
            else:
            	r.set("mode","camera_ready")
            
            time.sleep(2)

    def adj_bounding_boxes_callback(self, msg):
        if r.get("mode")=="adjust_ready":
            mushroom_xyz=msg.data
            mushroom_xyz=msg.data.split(",");
            goal=[int(float(mushroom_xyz[2].strip())),0-int(float(mushroom_xyz[0].strip()))]
            self.robot.get_scara_param()
            ret=self.robot.movej_xyz(self.robot.x-goal[0],self.robot.y-goal[1],self.robot.z,-180,80,1)
            self.get_logger().info(f"adj bounding_boxes_callback ->adjust :{goal},ret:{ret}")
            self.robot.wait_stop()
            if ret<2:
                r.set("mode","adjust_done")
                response = self.client_node.open_send_request()
            	#if response is not None:
                self.robot.get_scara_param()
                self.robot.wait_stop()
                ret=self.robot.movej_xyz(self.robot.x,self.robot.y,self.robot.z-120,self.robot.r,80,1)
                self.robot.wait_stop()
                self.get_logger().info(f"move in -z, ret :{ret}")
                response = self.client_node.send_request() #close request
                time.sleep(3)
                self.robot.get_scara_param()
                self.robot.wait_stop()
                self.client_node.get_logger().info(f'open 2 for {self.robot.z}')
                for i in range(3):
                    ret=self.robot.movej_xyz(self.robot.x,self.robot.y,self.robot.z,self.robot.r-3,30,1)
                    self.robot.wait_stop()
                    self.robot.get_scara_param()
                    self.robot.wait_stop()
                    ret=self.robot.movej_xyz(self.robot.x,self.robot.y,self.robot.z,self.robot.r+3,30,1)
                    self.robot.wait_stop()
                    time.sleep(0.5)
            	
            	#if response is not None:
                self.robot.get_scara_param()
                ret=self.robot.movej_xyz(self.robot.x,self.robot.y,0,self.robot.r,80,1)
                self.robot.wait_stop()
                time.sleep(1)
                ret=self.robot.movej_xyz(0,400,0,+20,50,1)
                self.robot.wait_stop()
                response = self.client_node.open_send_request()
            #else:
            # 	r.set("mode","camera_ready")
            r.set("mode","camera_ready")          
            time.sleep(2)

    def hitbot_gripper_adjust_done_callback(self, msg):
        self.get_logger().info(f"hitbot_gripper_adjust_done_callback:{msg}")
        if  r.get("mode")=="adjust_done":
            response = self.client_node.open_send_request()
            if response is not None:
                self.robot.get_scara_param()
                self.robot.wait_stop()
                ret=self.robot.movej_xyz(self.robot.x,self.robot.y,self.robot.z-180,self.robot.r,80,1)
                self.robot.wait_stop()
                self.get_logger().info(f"move in -z, ret :{ret}")
                self.client_node.get_logger().info(f'open for {response}')
                response = self.client_node.send_request() #close request
                time.sleep(2)
                if response is not None:
                    ret=self.robot.movej_xyz(self.robot.x,self.robot.y,0,self.robot.r,80,1)
                    self.robot.wait_stop()
                    time.sleep(1)
                    ret=self.robot.movej_xyz(0,400,0,+20,50,1)
                    self.robot.wait_stop()
                    response = self.client_node.open_send_request()
            else:
                self.get_logger().error('Service call failed %r' % (self.client_node.future.exception(),))

        r.set("mode","camera_ready") 

    def hitbot_gripper_adjust_callback(self, msg):
        xy=msg.data.split(",")
        leng=len(xy)
        self.get_logger().info(f'xy={xy},len={leng}')
        mode=r.get("mode")
        #adj_xy=[int(float(xy[0])),int(float(xy[0]))]
        if len(xy)>1:
            x_0,y_0,_,_=self.rt.tranform_point3_0([int(xy[0])-0.5,0.5-int(xy[1])],[self.robot.angle1*3.14/180,self.robot.angle2*3.14/180,self.robot.r*3.14/180])
            if abs(x_0)>50 or abs(y_0)>50:
                self.get_logger().info(f'xy={xy},mode={mode},pixel x_0:{x_0},pixel y0:{y_0}')
                #self.get_logger().info(f"adjust,current pos: x={self.goal[0]*1000},y={self.goal[1]*1000},target goal:{adj_goal}")
                #current_angle=[self.robot.angle1 *3.1415/180,self.robot.angle2*3.1415/180,self.robot.r*3.1415/180]
                #angles=self.solver.get_robot_angle_in_degree(adj_goal,current_angle)
                #self.get_logger().info(f"adjust,Computed   angle:{angles},current_angle:{self.robot.angle1},{self.robot.angle2},{self.robot.r}")
                #computed_pos=self.forward_kinematics_from_angles(angles[:3])
                #self.get_logger().info(f"adjust,Computed   position:{computed_pos}")
                self.robot.get_scara_param()
                self.robot.wait_stop()
                #ret=self.robot.movej_angle(angles[0],angles[1],0,angles[2]-180,50,1)
                #adj_goal=[(self.robot.x+x_0/30),(self.robot.y+y_0/30)]
                adj_goal=[(self.robot.x-int(xy[1])/10),(self.robot.y-int(xy[0])/10)] 
                
                ret=self.robot.movej_xyz(adj_goal[0],adj_goal[1],self.robot.z,-180,30,1)
                self.robot.wait_stop()
                r.set("mode","ready_to_adjust")#
                time.sleep(1)   
            else:
            	r.set("mode","adjust_done")
            	done=String()
            	done.data="done"
            	self.gripper_adj_done_pub.publish(done)            

        else:
            r.set("mode","camera_ready") #adjust_done
            done=String()
            done.data="done"
            self.gripper_adj_done_pub.publish(done)

    	#self.get_logger().info("xy pixel offset:{}".format(xy))  


    def joint_state_cb(self, msg):
        if not self.recording:
            return
            
        # Extract joint positions in correct order
        qpos = np.zeros(4)
        for i, name in enumerate(self.joint_names):
            idx = msg.name.index(name)
            qpos[i] = msg.position[idx]
        
        # Compute cartesian position (forward kinematics)
        cartesian_pos = self.forward_kinematics(qpos)
        #self.get_logger().info(f"cartesian_pos:{cartesian_pos}")
        # Compute action (delta from last position)
        if self.last_qpos is not None:
            action = qpos - self.last_qpos
            action = cartesian_pos - self.last_cartesian_pos
            self.buffer['action'].append(action)
        
  
    def image_cb(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        #act_tensor = self.prepare_act_input(cv_image)
        self.latest_image=cv_image #act_tensor
        if not self.recording:
            return
        

    def forward_kinematics(self, qpos):
        """Simple SCARA forward kinematics"""
        θ1, θ2, d, θ4 = qpos
        x = self.robot.angle1 #0.325 * np.cos(θ1) + 0.275 * np.cos(θ2)+0.17 * np.cos(θ4)  # Modify with your arm's lengths
        y = self.robot.angle2 # 0.325 * np.sin(θ1) + 0.275 * np.sin(θ2)+0.17 * np.sin(θ4)
        z = 0#self.robot.z  # Prismatic joint
        #rz = θ1 + θ2 + θ4  # Total rotation
        return np.array([x, y, z])

    
    def stop_and_save(self):
        if not self.recording:
            return
            
        self.recording = False
        
        # Ensure all buffers have same length
        min_length = min(len(self.buffer['observations']['images']),
                        len(self.buffer['observations']['qpos']),
                        len(self.buffer['action']))
                
        self.get_logger().info(f"min_length{min_length}")
        # Trim buffers
        for k in self.buffer['observations']:
            self.buffer['observations'][k] = list(self.buffer['observations'][k])[:min_length]
        self.buffer['action'] = list(self.buffer['action'])[:min_length]
        self.buffer['timestamps'] = list(self.buffer['timestamps'])[:min_length]
        
        # Save to HDF5
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        filename = os.path.join("/home/a/Downloads/mushroomproject/ros2_ws/", f"episode_{self.episode_count}.hdf5")
        
        with h5py.File(filename, 'w') as f:
            # Create observations group
            obs_group = f.create_group('observations')
            data = np.array([x.cpu().numpy() for x in self.buffer['observations']['images']])
		
            obs_group.create_dataset('images0', 
                                  data=np.array(data), #self.buffer['observations']['images']
                                  compression='gzip')
            obs_group.create_dataset('qpos',
                                  data=np.array(self.buffer['observations']['qpos']),
                                  compression='gzip')
            obs_group.create_dataset('cartesian_pos',
                                  data=np.array(self.buffer['observations']['cartesian_pos']),
                                  compression='gzip')
            obs_group.create_dataset('gripper',
                                  data=np.array(self.buffer['observations']['gripper']),
                                  compression='gzip')
            
            # Save actions and timestamps
            f.create_dataset('action',
                           data=np.array(self.buffer['action']),
                           compression='gzip')
            f.create_dataset('timestamps',
                           data=np.array(self.buffer['timestamps']),
                           compression='gzip')
            
            # Metadata
            f.attrs['robot_type'] = 'SCARA'
            f.attrs['joint_names'] = self.joint_names
            f.attrs['episode_id'] = self.episode_count
            
        self.episode_count += 1
        self.get_logger().info(f"Saved episode {filename}")

    def get_occupied_voxels(self,octree):
        self.occupied_voxels = set()  # Use a set for faster lookups
        for node in octree.begin_leafs():
            if octree.isNodeOccupied(node):
                x, y, z = node.getCoordinate()
                self.occupied_voxels.add((x, z))  # Store XZ coordinates
        return self.occupied_voxels

    
    def load_octomap(self,file_path):
        occupancy_map = octomap.OcTree(0.05)  #self.process_octomap(msg)# 5 cm resolution
        if isinstance(file_path, bytes):  
            file_path = file_path.decode("utf-8")  # ? Convert bytes to string

            # ? Load the Octomap from the file
        if occupancy_map.readBinary(file_path.encode()):  
            print("? Octomap loaded successfully!")
            print(f"Number of occupied nodes: {sum(1 for _ in occupancy_map.begin_leafs())}")
        else:
            print("? Error: Could not load Octomap from file!")
        return occupancy_map
    def octomap_callback(self, msg):
        """ Callback to receive and process Octomap data. """


        file_path="/home/a/Downloads/newmap.bt"
        octomap_size = len(msg.data)
        self.get_logger().info(f"octomap_size: {octomap_size},{self.robot.x/1000}{self.robot.y/1000},{self.robot.z/1000}")
        if not msg.data:
            return
        output=subprocess.check_output(["ros2", "run", "octomap_server", "octomap_saver_node", "--ros-args", "-p" ,"octomap_path:=/home/a/Downloads/newmap.bt" ,"-r" ,"octomap_binary:=/rtabmap/octomap_binary" ],text=True)



        self.occupancy_map = self.load_octomap(file_path)  #self.process_octomap(msg)# 5 cm resolution

        if self.occupancy_map:
            start = ( self.robot.x/1000,(self.robot.y)/1000, 0.0)  # Example start position
            goal = ((self.robot.x-100)/1000,(self.robot.y+50)/1000, 0.0 )   # Example goal position
            #self.convert_to_fcl_objects()#fcl.OcTree(self.occupancy_map)  # ? Convert Octomap to FCL Octree
    
            # Define joint positions (example)
            target_position = list(goal)  # Replace with your target position
            target_orientation = [0, 0, 0.0]  # Replace with your target orientation (roll, pitch, yaw)
            
            joint_angles = self.scara_arm.inverse_kinematics(target_position,target_orientation)
            print("Computed Joint Angles:", joint_angles)

            # Compute forward kinematics to get the positions of all joints
            joint_positions = self.scara_arm.forward_kinematics(joint_angles, full_kinematics=True)

            # Extract the positions of all joints
            joint_positions = [frame[:3, 3] for frame in joint_positions]

            camera_joint_positions=self.R@np.transpose(np.matrix(joint_positions))
            camera_joint_positions=np.asarray(np.transpose(camera_joint_positions))
            print("Computed Joint joint_positions:",camera_joint_positions)
        
            occupied_voxels = self.get_occupied_voxels(self.occupancy_map)
            # Check for collisions
            collision=self.check_robot_collision_xz( camera_joint_positions, occupied_voxels)
            if(collision):
                return
            self.get_logger().info(f"before rrt planning")
            path = self.rrt_planning(start, goal, max_iter=500)

            if path:
                self.get_logger().info(f"get apath {path}")
                self.visualize_path(path)
            else:
                self.get_logger().warn("No valid path found.")


    def hitbot_end_xyzr_callback(self,msg):
        xyzr=msg.data.split(",");
        self.get_logger().info(f'hitbot_end_xyzr_callback:{msg},{xyzr}')
        ret=self.robot.movel_xyz(int(xyzr[0]),int(xyzr[1]),int(xyzr[2]),int(xyzr[3]),100)
        self.get_logger().info(f"movel_xyz ret: {ret}")
        self.robot.wait_stop()
    def hitbot_end_angle_callback(self,msg):
        self.get_logger().info(f'hitbot_end_angle_callback:{msg}')
        angles=msg.data.split(",");
        self.get_logger().info(f'hitbot_end_angle_callback:{msg},{angles}')
        angle1=int(angles[0])
        angle2=int(angles[1])
        z=int(angles[2])
        r=int(angles[3])
        ret=self.robot.movej_angle(angle1,angle2,z,r,50,1)
        self.get_logger().info(f"movel_xyz ret: {ret}")
        self.robot.wait_stop()
        
    def hitbot_move_z_callback(self,msg):
        self.get_logger().info(f'hitbot_move_z_callback:{msg}')
        z=msg.data
        self.get_logger().info(f'hitbot_move_z_callback:{msg}')
        ret=self.robot.movej_xyz(self.robot.x,self.robot.y,self.robot.z-z,self.robot.r,100,1)

        self.get_logger().info(f"hitbot_move_z_callback ret: {ret}")
        self.robot.wait_stop()
    
    

    def publish_joint_states(self):
        # Get real joint positions from HitBot API (Replace this with actual API calls)
        self.robot.get_scara_param()
        joint_positions = [self.robot.z,self.robot.angle1*3.14/180,self.robot.angle2*3.14/180,(self.robot.r-48)*3.14/180]

        # Create JointState message
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = self.joint_names
        joint_state_msg.position = joint_positions

        self.hitbot_x=self.robot.x/1000
        self.hitbot_y=self.robot.y/1000
        self.hitbot_z=self.robot.z/1000


        # Publish the joint states
        self.joint_state_pub.publish(joint_state_msg)

        self.publish_hitbot_x(str(int(self.robot.x)))
        self.publish_hitbot_y(str(int(self.robot.y)))
        self.publish_hitbot_z(str(int(self.robot.z)))
        #self.get_logger().info(f"publish_joint_states str(int(self.robot.z)) {str(int(self.robot.z))}")
        #self.publish_hitbot_r(str(int(self.robot.r-48)))
        camera_xyz=String()
        camera_xyz.data=str(self.robot.x)+","+str(self.robot.y)
        self.camera_xyz_publisher.publish(camera_xyz)
        r.set("global_camera_xy",camera_xyz.data)

       
        
        #self.get_logger().info(f"Published joint states: {joint_state_msg}")
    def joint_command_callback(self, msg):

        try:
            positions = []
            #joint_limits = [(-1.2, 0.0), (-1.571, 1.571), (-2.967, 2.967), (-18.850, 18.850)]
            trajs=msg.trajectory[-1].joint_trajectory.points
            self.get_logger().info(f"joint_command_callback trajectory: {trajs}")
            for i in range(0, len(trajs)):
                waypoints = trajs[i].positions
                self.get_logger().info(f"i={i},waypoints : {waypoints}")
                #for j in range(0, len(waypoints)):
                  #positions=waypoints[j].positions
                self.robot.new_movej_angle(waypoints[1]*180/3.14, waypoints[2]*180/3.14, waypoints[0], (waypoints[3]*180/3.14)-48.0, 50, 1)
                #
            self.robot.wait_stop()

                #if pos < min_limit or pos > max_limit:
                #    print(f"Position for joint{i} must be between {min_limit} and {max_limit}.")
                #    return self.get_positions_from_user()
                #positions.append(pos)
                

            #return positions
        except ValueError:
            print("Invalid input. Please enter numerical values.")


        self.get_logger().info(f"Sent joint command to HitBot")    

    def publish_hitbot_x(self, data):
        msg = String()

        msg.data = (data)
        self.hitbot_x_publisher.publish(msg)

    def publish_hitbot_y(self, data):
        msg = String()
        msg.data = (data)

        self.hitbot_y_publisher.publish(msg)

    def publish_hitbot_z(self, data):
        msg = String()
        msg.data = (data)

        self.hitbot_z_publisher.publish(msg)

    def publish_hitbot_r(self, data):
        msg = String()
        msg.data = (data)
        self.hitbot_r_publisher.publish(msg)



  

    def joint_home_callback(self, request, response):
        max_retries = 3
        retries = 0
        
        while retries < max_retries:
            try:
                self.robot.joint_home(request.joint_num)
                response.success = True
                if self.robot.joint_home(request.joint_num) == 0:
                    self.get_logger().info('Joint Not connected. Check your robot')
                elif self.robot.joint_home(request.joint_num) == 1:
                    self.get_logger().info('Success')
                elif self.robot.joint_home(request.joint_num) == 2:
                    self.get_logger().info('Invalid parameters passed')
                elif self.robot.joint_home(request.joint_num) == 3:
                    self.get_logger().info('The robot is still initializing.')
                else:
                    pass
                break
            except Exception as e:
                self.get_logger().error('Failed to joint home: %s' % str(e))
                response.success = False
                retries += 1
                if retries < max_retries:
                    self.get_logger().info('Retrying joint home (attempt %d)...' % retries)
                else:
                    self.get_logger().error('Max retries exceeded. Failed to joint home.')
                    break
        return response

    def new_movej_xyz_lr_callback(self, request, response):
        max_retries = 3
        retries = 0
        
        while retries < max_retries:
            try:
                self.robot.new_movej_xyz_lr(request.goal_x, request.goal_y, request.goal_z, request.goal_r, request.speed, request.roughly, request.lr)
                response.success = True
                if self.robot.new_movej_xyz_lr(request.goal_x, request.goal_y, request.goal_z, request.goal_r, request.speed, request.roughly, request.lr) == 1:
                    self.get_logger().info('Robot Moving')
                else:
                    self.get_logger().info('Error, Check Robot')
                break
            except Exception as e:
                self.get_logger().error('Failed to new_movej_xyz_lr: %s' % str(e))
                response.success = False
                retries += 1
                if retries < max_retries:
                    self.get_logger().info('Retrying new_movej_xyz_lr (attempt %d)...' % retries)
                else:
                    self.get_logger().error('Max retries exceeded. Failed to new_movej_xyz_lr.')
        return response


    def new_movej_angle_callback(self, request, response):
        max_retries = 3
        retries = 0
        
        while retries < max_retries:
            try:
                self.robot.movej_angle(request.goal_angle1, request.goal_angle2, request.goal_z, request.goal_r, request.speed, request.roughly)
                response.success = True
                if self.robot.movej_angle(request.goal_angle1, request.goal_angle2, request.goal_z, request.goal_r, request.speed, request.roughly) == 1:
                    self.get_logger().info('Robot Moving')
                else:
                    self.get_logger().info('Error, Check Robot')
                break
            except Exception as e:
                self.get_logger().error('Failed to new_movej_angle: %s' % str(e))
                response.success = False
                retries += 1
                if retries < max_retries:
                    self.get_logger().info('Retrying new_movej_angle (attempt %d)...' % retries)
                else:
                    self.get_logger().error('Max retries exceeded. Failed to new_movej_angle.')
        return response

    def init_robot(self):
        self.robot.net_port_initial()
        time.sleep(1)
        is_connected = self.robot.is_connect()

        if is_connected != 1:
            print('No robot connection!!!')
            raise RuntimeError('No robot connection!!!')

        print('Robot connected.')

        init = self.robot.initial(1, 210) ## 1000 is z-axis parameter, Modify it to your own

        if init != 1:
            print('Robot initialization failed!!!')
            raise RuntimeError('Robot initialization failed!!!')

        print('unlock Robot')
        self.robot.unlock_position()
        print('Robot position initialized.')
        ret=self.robot.movel_xyz(600,0,0,-180,50)
    	#ret=self.robot.new_movej_xyz_lr(hi.x-100,hi.y,0,-180,20,0,1)
        #ret=self.robot.movej_angle(0,30,0,0,20,0)
        self.robot.wait_stop()
        r.set("mode","camera_ready") 
        print('Robot I/O output initialized.')
        for i in range(12):
            self.robot.set_digital_out(i, False)
        time.sleep(1)
        print('Robot initialized.')
    
    #def run(self):
    #    print("hello hibot")
    #    # 使用多线程执行器
    #    executor = MultiThreadedExecutor(num_threads=2)
    #    executor.add_node(self)

    #    try:
    #        executor.spin()
    #    except Exception as e:
    #        print("Executor error:", str(e))
    #    finally:
    #        self.destroy_node()
    #        rclpy.shutdown()
    def run(self):
        print("hello hibot")

        while rclpy.ok():
            try:
                rclpy.spin_once(self)
            except ValueError as e:
                print("Error:", str(e))
            except RuntimeError as e:
                print("Error:", str(e))

        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)

    hitbot_controller = HitbotController()

    try:
        hitbot_controller.run()
    except KeyboardInterrupt:
        print("KeyboardInterrupt")
    finally:
        hitbot_controller.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
