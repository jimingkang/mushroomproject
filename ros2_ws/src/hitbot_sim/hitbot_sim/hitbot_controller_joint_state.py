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


redis_server='localhost'#'172.27.34.62'
pool = redis.ConnectionPool(host=redis_server, port=6379, decode_responses=True,password='jimmy')
r = redis.Redis(connection_pool=pool)

os.chdir(os.path.expanduser('~'))
#sys.path.append("./ws_moveit/src/hitbot")  ## get import pass: hitbot_interface.py
#from .hitbot_interface import HitbotInterface
from .HitbotInterface import HitbotInterface


import numpy as np
from scipy.optimize import root
from matplotlib import pyplot as plt

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
        return solved[min_config]
    def get_robot_angle_in_degree(self, target, current_angle=[0,0,0]):
        ans=self.get_angle_to_target(target, current_angle)
        return [round(ans[0]*180/3.1415,0),round(ans[1]*180/3.1415,0),round((ans[0]+ans[1]+ans[2])*180/3.1415,0)]


class ServiceClient(Node):
    def __init__(self):
        super().__init__('service_client')
        self.client = self.create_client(Trigger, 'close_gripper_service')  # Service type and name
        self.open_client = self.create_client(Trigger, 'open_gripper_service')  # Service type and name
       
        # Wait for service to be available
        #while not self.client.wait_for_service(timeout_sec=1.0):
        #    self.get_logger().info('close service not available, waiting again...')
        #while not self.open_client.wait_for_service(timeout_sec=1.0):
        #    self.get_logger().info('open service not available, waiting again...')
       
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
        self.config = {
        'ckpt_dir': './',
        'ckpt_name': 'policy_best.ckpt',
        'policy_class': 'ACT',
        'policy_config': {
                        'num_queries': 5,
 'lr':1e-5,# args['lr'],
                         'num_queries':100,# args['chunk_size'],
                         'kl_weight':10,# args['kl_weight'],
                         'hidden_dim': 512,#args['hidden_dim'],
                         'dim_feedforward':3200,# args['dim_feedforward'],
                         'lr_backbone': 1e-5,
                         'backbone': 'resnet18',
                         'enc_layers': 4,
                         'dec_layers': 7,
                         'nheads': 8,
                         'camera_names': 'top',
        },
        'state_dim': 4,
        'camera_topic': '/camera/image_raw',
        'joint_topic': '/joint_states',
        'command_topic': '/joint_commands',
        'reset_position': np.array([0.0, 0.0, 0.0, 0.0]),
        'control_freq': 10.0,  #192.168.0.100 Hz
        'success_threshold': 1.0,
        }
        #self.policy = self.load_policy()
        #self.stats = self.load_stats()
        # Pre/post-processing functions
        #self.pre_process = lambda s: (s - self.stats['qpos_mean']) / self.stats['qpos_std']
        #self.post_process = lambda a: a * self.stats['action_std'] + self.stats['action_mean']
        r.set("mode","camera_ready")

        self.client_node = ServiceClient()
        self.solver=Solver()


        self.SetGPIO_srv = self.create_service(SetGPIO, 'set_gpio', self.set_gpio_callback)
        self.GetGPIOOut_srv = self.create_service(GetGPIOOut, 'get_gpio_out', self.get_gpio_out_callback)
        self.GetGPIOIn_srv = self.create_service(GetGPIOIn, 'get_gpio_in', self.get_gpio_in_callback)
        self.SetDragTeach_srv = self.create_service(SetDragTeach, 'set_drag_teach', self.set_drag_teach_callback)
        self.JointHome_srv = self.create_service(JointHome, 'joint_home', self.joint_home_callback)
        self.NewMovejXYZ_srv = self.create_service(NewMovejXYZ, 'new_movej_xyz_lr', self.new_movej_xyz_lr_callback)
        self.NewMovejAngle_srv = self.create_service(NewMovejAngle, 'new_movej_angle', self.new_movej_angle_callback)


        self.hitbot_x = 0
        self.hitbot_y = 0
        self.hitbot_z = 0
        self.hitbot_r = -48

        self.hitbot_t1_publisher = self.create_subscription(Int32, '/hitbot_theta1',self.hitbot_theta1_callback, 10)
        self.hitbot_t2_publisher = self.create_subscription(Int32, '/hitbot_theta2',self.hitbot_theta2_callback, 10)
        self.hitbot_t2_publisher = self.create_subscription(Int32, '/hitbot_theta3',self.hitbot_theta3_callback,10)
        #self.hitbot_t3_publisher = self.create_subscription(String, '/hitbot_theta3', self.hitbot_theta4_callback,10)

        self.hitbot_x_publisher = self.create_publisher(String, '/hitbot_x', 10)
        self.hitbot_y_publisher = self.create_publisher(String, '/hitbot_y', 10)
        self.hitbot_z_publisher = self.create_publisher(String, '/hitbot_z', 10)
        self.hitbot_r_publisher = self.create_publisher(String, '/hitbot_r', 10)
        self.camera_xyz_publisher = self.create_publisher(String, '/camera_xyz', 10)


        self.bounding_boxes_sub = self.create_subscription(String,"/yolox/bounding_boxes",self.bounding_boxes_callback, 10)
        self.xyz_sub = self.create_subscription(String,"/hitbot_end_xyz",self.hitbot_end_xyzr_callback,10)
        self.angle_sub = self.create_subscription(String,"/hitbot_end_angle",self.hitbot_end_angle_callback,10)
        

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

        self.urdf_file = "/home/a/Downloads/mushroomproject/ros2_ws/build/hitbot_sim/hitbot_sim/scara_ik.xml"
        self.scara_arm = Chain.from_urdf_file(self.urdf_file)
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
        r.set("mode","pickup_ready")
        mushroom_xyz=msg.data
        mushroom_xyz=msg.data.split(",");
        self.get_logger().info(f"get mushroom_xyz:{mushroom_xyz}")
        goal=[int(float(mushroom_xyz[2].strip()))/1000,0-int(float(mushroom_xyz[0].strip()))/1000]
        self.get_logger().info(f"target get goal:{goal}")
        current_angle=[self.robot.angle1 *3.1415/180,self.robot.angle2*3.1415/180,self.robot.r*3.1415/180]
        angles=self.solver.get_robot_angle_in_degree(goal,current_angle)
        #angles=self.ik_solver.inverse_kinematics([goal[0]/1000,goal[1]/1000,0])
        self.get_logger().info(f"Computed   angle:{angles}")
        computed_pos=self.forward_kinematics_from_angles(angles[:3])
        self.get_logger().info(f"Computed   position:{computed_pos}")
        if (abs(angles[2]))*180/3.14>100:
            r.set("mode","camera_ready")
            self.get_logger().info(f"self collide")
            #return

        ret=self.robot.movej_angle(angles[0],angles[1],0,angles[2]-20,100,1) 
        #ret=self.robot.movej_xyz(int(float(mushroom_xyz[2].strip())-230),100-int(float(mushroom_xyz[0].strip())),0,-48,100,1)
        self.get_logger().info(f"ret :{ret}")
        self.robot.wait_stop()


        #response = self.client_node.open_send_request()
        if 0:#response is not None:
            self.robot.get_scara_param()
            self.robot.wait_stop()
            ret=self.robot.movej_xyz(self.robot.x,self.robot.y,self.robot.z-110,-48,100,1)
            self.robot.wait_stop()
            self.client_node.get_logger().info(f'open for {response}')
            response = self.client_node.send_request()
            if response is not None:
                time.sleep(3)
                ret=self.robot.movej_xyz(self.robot.x,self.robot.y,0,-48,100,1)
                self.robot.wait_stop()
                time.sleep(1)
                ret=self.robot.movej_xyz(0,-400,0,-48-230,100,1)
                self.robot.wait_stop()
                response = self.client_node.open_send_request()


        #else:
        #    self.get_logger().error('Service call failed %r' % (self.client_node.future.exception(),))
        self.get_logger().info(f"move in -z, ret :{ret}")



        r.set("mode","camera_ready")
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
        
        # Store observations
        self.buffer['observations']['qpos'].append(qpos)
        self.buffer['observations']['cartesian_pos'].append(cartesian_pos)
        self.buffer['observations']['gripper'].append(qpos[-1])  # Last joint is gripper
        self.buffer['timestamps'].append(msg.header.stamp.to_sec())
        
        self.last_qpos = qpos
    def prepare_act_input(self, cv_image):
        #self.get_logger().info(f"cv_image:{len(cv_image.shape)}")
        """Convert OpenCV image to ACT policy input tensor"""
        # 1. Ensure 3-channel RGB
        if len(cv_image.shape) == 2:  # Grayscale
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2RGB)
        elif cv_image.shape[2] == 4:  # RGBA
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGBA2RGB)
        elif cv_image.shape[2] == 3:  # Check if BGR
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        
        # 2. Resize to expected dimensions (typically 224x224)
        cv_image = cv2.resize(cv_image, (224, 224))
        # 3. Convert to float32 and normalize [0,1]
        cv_image = cv_image.astype(np.float32) / 255.0
        # 4. Convert HWC to CHW and to tensor
        tensor = torch.from_numpy(cv_image.transpose(2, 0, 1)).float()
        
        # 5. Normalize with ImageNet stats
        tensor = (tensor.cuda() - self.mean) / self.std
        
        # 6. Add batch dimension [1, 3, 224, 224]
        return tensor.unsqueeze(0)
    def image_cb(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        #act_tensor = self.prepare_act_input(cv_image)
        self.latest_image=cv_image #act_tensor
        if not self.recording:
            return
        try:

            qpos = np.zeros(4)
            qpos[0]=self.robot.x  #.x
            qpos[1]=self.robot.y  #.y
            qpos[2]=self.robot.z
            qpos[3]=0#(self.robot.r+48)
            
            # Compute cartesian position (forward kinematics)
            cartesian_pos = self.forward_kinematics(qpos)
            self.get_logger().info(f"cartesian_pos:{cartesian_pos}")
            
            # Compute action (delta from last position)
            if self.last_qpos is not None:
                action = qpos - self.last_qpos             
                self.buffer['action'].append(action)
            
            # Store observations
            self.buffer['observations']['qpos'].append(qpos)
            self.buffer['observations']['cartesian_pos'].append(cartesian_pos)
            self.buffer['observations']['gripper'].append(qpos[-1])  # Last joint is gripper
            self.buffer['timestamps'].append( msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)        
            self.last_qpos = qpos
            self.last_cartesian_pos=cartesian_pos
            self.buffer['observations']['images'].append(self.latest_image)

        except Exception as e:
            self.get_logger().error(e)

    def forward_kinematics(self, qpos):
        """Simple SCARA forward kinematics"""
        θ1, θ2, d, θ4 = qpos
        x = self.robot.angle1 #0.325 * np.cos(θ1) + 0.275 * np.cos(θ2)+0.17 * np.cos(θ4)  # Modify with your arm's lengths
        y = self.robot.angle2 # 0.325 * np.sin(θ1) + 0.275 * np.sin(θ2)+0.17 * np.sin(θ4)
        z = 0#self.robot.z  # Prismatic joint
        #rz = θ1 + θ2 + θ4  # Total rotation
        return np.array([x, y, z])

    def start_recording(self):
        self.recording = True
        self.last_qpos = None
        self.buffer = {k: {sk: deque(maxlen=1000) for sk in v} if isinstance(v, dict) else deque(maxlen=1000) 
                     for k, v in self.buffer.items()}
        self.get_logger().info("Started recording new episode")

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

    def check_joint_collision(self,p1, occupied_voxels, voxel_size=0.1):
        # Bresenham's line algorithm to check for intersections
        #x1, z1 = p1

        nearest_node, distance = self.find2_nearest_node(p1, occupied_voxels)
        print(distance)
        if (distance is not None) and (distance<0.1):
            return True
        else: 
            return False

        return nearest_node, min_distance
# Check if the robot's links collide with the obstacles in the XZ plane
    def check_robot_collision_xz(self, joint_positions, occupied_voxels):
        # Get the transformation matrices for each joint
        #frames = chain.forward_kinematics(joint_positions, full_kinematics=True)
        # Extract the XZ positions of the joints
        #joint_positions_xz = [frame[[0, 2], 3] for frame in frames]  # Extract X and Z coordinates
        # Extract the XY positions of the joints
        #joint_positions_xy = [frame[:2, 3] for frame in frames]  # Extract X and Y coordinates

        joint_positions_xz =joint_positions#np.array(joint_positions)[:,:2].tolist() # Extract X and Y coordinates
        # Check for collisions between each pair of consecutive joints
        for i in range(len(joint_positions_xz) - 1):
            p1 = joint_positions_xz[i]
            p2 = joint_positions_xz[i + 1]
            if self.check_joint_collision(p1, occupied_voxels):
                print(f"Collision detected between joint {i} and joint {i + 1}!")
                return True
            #if check_line_collision(p1, p2, occupied_voxels):
            #    print(f"Collision detected between joint {i} and joint {i + 1}!")
            #    return True
        print("No collision detected.")
        return False
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


    def convert_to_fcl_objects(self):
        """ Convert Octomap Occupied Nodes into FCL Collision Objects """
        if not self.occupancy_map:
            self.get_logger().error("? No Octomap data available!")
            return

        occupied_voxels= []  # Clear old objects
        self.get_logger().info(f"? Converted {len(self.fcl_octree)} occupied nodes into FCL objects!")





    def find_nearest_node(self, query_point, find_occupied=True):

        nearest_node = None
        min_distance = 0.5

        # Iterate through all leaf nodes in the OctoMap
        for node in self.occupancy_map.begin_leafs():
            node_coord = node.getCoordinate()
            node_occupancy = node.getOccupancy()  # Occupancy probability (0=free, 1=occupied)
            
            # Check if the node matches the requested type (occupied or free)
            is_occupied = node_occupancy > 0.5  # Adjust threshold if necessary
            if is_occupied != find_occupied:
                continue

            # Compute Euclidean distance
            node_point = np.array([node_coord[0],  node_coord[2]])
            query_point_np = np.array(query_point)
            distance = np.linalg.norm(node_point - query_point_np)

            # Update nearest node if closer
            if distance < min_distance:
                nearest_node = node_point
                min_distance = distance

        return (tuple(nearest_node), min_distance) if nearest_node is not None else (None, None)


    def is_collision_free(self, point):
        """ Check if a point is collision-free in the Octomap. """
        if self.occupancy_map is None:
            return True  # If no map, assume free space
        
        x, y, z = point
        #print(point)
        target_position=[x,y,z]
        joint_angles = self.scara_arm.inverse_kinematics(target_position)
        #print("  Computed Joint Angles:", joint_angles)

        #Compute forward kinematics to get the positions of all joints
        joint_positions = self.scara_arm.forward_kinematics(joint_angles, full_kinematics=True)

        # Extract the positions of all joints
        joint_positions = [frame[:3, 3] for frame in joint_positions]

        camera_joint_positions=self.R@np.transpose(np.matrix(joint_positions))
        camera_joint_positions=np.asarray(np.transpose(camera_joint_positions))
        #print(" Computed camera_joint_positions:",camera_joint_positions)
        
        occupied_voxels = self.get_occupied_voxels(self.occupancy_map)
            # Check for collisions
        collision=self.check_robot_collision_xz( camera_joint_positions, occupied_voxels)
        if collision:
            return False
        else:
            return True



    def rrt_planning(self, start, goal, max_iter=1000, step_size=0.05):
        """ RRT Path Planning from `start` to `goal` avoiding obstacles. """
        nodes = [start]  # RRT Tree
        parents = {start: None}  # Parent dictionary
        #self.get_logger().info(f" in rrt_planning:{parents}")
        for _ in range(max_iter):
            rand_point = self.random_point(goal)
            nearest_node = self.nearest_neighbor(rand_point, nodes)
            new_node = self.steer(nearest_node, rand_point, step_size)

            #self.get_logger().info(f" beofre is_collision_free")
            if self.is_collision_free(new_node):
                nodes.append(new_node)
                parents[new_node] = nearest_node

                if np.linalg.norm(np.array(new_node) - np.array(goal)) < step_size:
                    return self.reconstruct_path(parents, new_node)

        return None  # No valid path found

    def random_point(self, goal, bias=0.1):
        """ Generate a random point in space with a goal bias. """
        if random.random() < bias:
            return goal  # Occasionally pick the goal to bias growth
        return (
            random.uniform(self.bounds[0], self.bounds[1]),
            random.uniform(self.bounds[2], self.bounds[3]),
            random.uniform(self.bounds[4], self.bounds[5])
        )

    def nearest_neighbor(self, point, nodes):
        """ Find the nearest node in the tree using KDTree. """
        tree = KDTree(nodes)
        _, idx = tree.query(point)
        return nodes[idx]

    def steer(self, from_node, to_node, step_size):
        """ Generate a new node in the direction of `to_node` from `from_node`. """
        direction = np.array(to_node) - np.array(from_node)
        length = np.linalg.norm(direction)
        if length < step_size:
            return to_node
        direction = direction / length
        new_point = np.array(from_node) + direction * step_size
        return tuple(new_point)

    def reconstruct_path(self, parents, end_node):
        """ Reconstruct the path from the end node to the start. """
        path = []
        node = end_node
        while node:
            path.append(node)
            node = parents[node]
        path.reverse()
        return path

    def visualize_path(self, path):
        """ Publish the planned path as a Marker in RViz. """
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.01  # Line width
        marker.color.a = 1.0  # Alpha
        marker.color.r = 1.0  # Red
        marker.color.g = 0.0  # Green
        marker.color.b = 0.0  # Blue

        for (x, y, z) in path:
            point = Point()
            point.x, point.y, point.z = x, y, z
            marker.points.append(point)
            self.robot.movej_xyz(x*1000,y*1000,0,-48,50,1)

        self.path_publisher.publish(marker)
        self.get_logger().info("Published RRT path.")

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
        self.publish_hitbot_r(str(int(self.robot.r-48)))
        camera_xyz=String()
        camera_xyz.data=str(self.robot.x)+","+str(self.robot.y)
        self.camera_xyz_publisher.publish(camera_xyz)
        r.set("global_camera_xy",camera_xyz.data)


        qpos = np.zeros(4)
        qpos[0]=self.robot.x
        qpos[1]=self.robot.y
        qpos[2]=self.robot.z
        qpos[3]=0#(self.robot.r+48)
        self.latest_qpos=qpos

        max_timesteps = 16
        temporal_agg = False
        query_frequency = 10
        if temporal_agg:
            num_queries = 100#policy_config['num_queries']
            all_time_actions = torch.zeros([max_timesteps, max_timesteps+num_queries, 4]).cuda()
        if 0:# (self.latest_image is not None):
            obs={'image': self.latest_image,'qpos': self.latest_qpos}
            self.get_logger().info(f"latest_qpos:{self.latest_qpos}")
            qpos = self.pre_process(obs['qpos'][:4])

            qpos_tensor = torch.from_numpy(qpos).float().cuda().unsqueeze(0)
            # Convert image
            image_tensor =self.latest_image #torch.from_numpy(obs['image'].transpose(2, 0, 1)).float().cuda().unsqueeze(0)
            image_tensor=image_tensor.cuda()

            with torch.no_grad():
                for t in range(max_timesteps):
                    if t % query_frequency == 0:
                        all_actions = self.policy(qpos_tensor, image_tensor)
                    if temporal_agg:
                        all_time_actions[[t], t:t+num_queries] = all_actions
                        actions_for_curr_step = all_time_actions[:, t]
                        actions_populated = torch.all(actions_for_curr_step != 0, axis=1)
                        actions_for_curr_step = actions_for_curr_step[actions_populated]
                        k = 0.01
                        exp_weights = np.exp(-k * np.arange(len(actions_for_curr_step)))
                        exp_weights = exp_weights / exp_weights.sum()
                        exp_weights = torch.from_numpy(exp_weights).cuda().unsqueeze(dim=1)
                        raw_action = (actions_for_curr_step * exp_weights).sum(dim=0, keepdim=True)
                        #self.get_logger().info(f" temporal_agg raw_action:{raw_action.shape},{raw_action}")   
                    else:
                        raw_action = all_actions[:, t % query_frequency]
 
            # Postprocess and send command
            raw_action = raw_action.cpu().numpy().squeeze()
            target_joints =self.post_process(raw_action)
            self.get_logger().info(f"target_joints:{target_joints[0]},{target_joints[1]},{target_joints[2]},{target_joints[3]}")
            #self.get_logger().info(f"total_joints:{self.robot.angle1+(target_joints[0])},{self.robot.angle2+(target_joints[1])},{self.robot.z+target_joints[2]},{self.robot.r+target_joints[3]}")            
            self.robot.get_scara_param()
            self.robot.wait_stop()
            ret=self.robot.movel_xyz((target_joints[0]),(target_joints[1]),target_joints[2],-48.0,20)            
            #ret=self.robot.new_movej_angle(self.robot.angle1+(target_joints[0]),self.robot.angle2+(target_joints[1]),0,target_joints[3]-48.0,50,1)
            
            print(f"ret:{ret}")
            self.robot.wait_stop()

            

        
        #self.get_logger().info(f"Published joint states: {joint_state_msg}")
    def joint_command_callback(self, msg):
        #self.get_logger().info(f"joint_command_callback trajectory: {msg}")
        #if not msg.points:
        #    return
        # Get the last trajectory point (final command position)
        #last_point = msg.points[-1]
        #joint_positions = last_point.positions  # Extract joint positions
        #hitbot_command_msg = Float64MultiArray()
        #hitbot_command_msg.data = list(joint_positions)  # Convert to list
        ##self.hitbot_command_pub.publish(hitbot_command_msg)
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




    def set_gpio_callback(self, request, response):
        max_retries = 3
        retries = 0
        
        while retries < max_retries:
            try:
                self.robot.set_digital_out(request.gpio_number, request.set_on)
                response.success = True
                self.get_logger().info('GPIO setting successful: gpio_number=%d, set_on=%r' % (request.gpio_number, request.set_on))
                break
            except Exception as e:
                self.get_logger().error('Failed to set GPIO: %s' % str(e))
                response.success = False
                retries += 1
                if retries < max_retries:
                    self.get_logger().info('Retrying GPIO setting (attempt %d)...' % retries)
                else:
                    self.get_logger().error('Max retries exceeded. Failed to set GPIO.')
                    break

        return response
    
    def get_gpio_out_callback(self, request, response):
        max_retries = 3
        retries = 0
        
        while retries < max_retries:
            try:
                self.robot.get_digital_out(request.gpio_number)
                response.success = True
                if self.robot.get_digital_out(request.gpio_number) == 1:
                    self.get_logger().info('GPIO : gpio_number=%d is On' % (request.gpio_number))
                elif self.robot.get_digital_out(request.gpio_number) == 0:
                    self.get_logger().info('GPIO : gpio_number=%d is Off' % (request.gpio_number))
                elif self.robot.get_digital_out(request.gpio_number) == -1:
                    self.get_logger().info('GPIO number parameter error')
                elif self.robot.get_digital_out(request.gpio_number) == 3:
                    self.get_logger().info('GPIO Not initialized.')
                else:
                    pass
                break
            except Exception as e:
                self.get_logger().error('Failed to get GPIO: %s' % str(e))
                response.success = False
                retries += 1
                if retries < max_retries:
                    self.get_logger().info('Retrying GPIO status (attempt %d)...' % retries)
                else:
                    self.get_logger().error('Max retries exceeded. Failed to get GPIO.')
                    break

        return response
    
    def get_gpio_in_callback(self, request, response):
        max_retries = 3
        retries = 0
        
        while retries < max_retries:
            try:
                self.robot.get_digital_out(request.gpio_number)
                response.success = True
                if self.robot.get_digital_out(request.gpio_number) == 1:
                    self.get_logger().info('GPIO : gpio_number=%d is triggered' % (request.gpio_number))
                elif self.robot.get_digital_out(request.gpio_number) == 0:
                    self.get_logger().info('GPIO : gpio_number=%d is not triggered' % (request.gpio_number))
                elif self.robot.get_digital_out(request.gpio_number) == -1:
                    self.get_logger().info('GPIO number parameter error')
                elif self.robot.get_digital_out(request.gpio_number) == 3:
                    self.get_logger().info('GPIO Not initialized.')
                else:
                    pass
                break
            except Exception as e:
                self.get_logger().error('Failed to get GPIO: %s' % str(e))
                response.success = False
                retries += 1
                if retries < max_retries:
                    self.get_logger().info('Retrying GPIO status (attempt %d)...' % retries)
                else:
                    self.get_logger().error('Max retries exceeded. Failed to get GPIO.')
                    break

        return response

    def set_drag_teach_callback(self, request, response):
        max_retries = 3
        retries = 0
        
        while retries < max_retries:
            try:
                self.robot.set_drag_teach(request.enable)
                response.success = True
                if self.robot.set_drag_teach(request.enable) == True:
                    self.get_logger().info('Successfully set drag teach')
                elif self.robot.set_drag_teach(request.enable) == False:
                    self.get_logger().info('Failed to set drag teach, please check your robot model enable drag teach')
                else:
                    pass
                break
            except Exception as e:
                self.get_logger().error('Failed to set drag teach: %s' % str(e))
                response.success = False
                retries += 1
                if retries < max_retries:
                    self.get_logger().info('Retrying set drag teach (attempt %d)...' % retries)
                else:
                    self.get_logger().error('Max retries exceeded. Failed to set drag teach.')
                    break

        return response

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
        #ret=self.robot.movel_xyz(600,0,0,-63,20)
    	#ret=self.robot.new_movej_xyz_lr(hi.x-100,hi.y,0,-63,20,0,1)
        #ret=self.robot.movej_angle(0,30,0,0,20,0)
        #self.robot.wait_stop()
        print('Robot I/O output initialized.')
        for i in range(12):
            self.robot.set_digital_out(i, False)
        time.sleep(1)
        print('Robot initialized.')
    def load_stats(self):
        """Load dataset statistics"""
        stats_path = os.path.join("/home/a/Downloads/mushroomproject/ros2_ws/src/hitbot_sim/hitbot_sim", 'dataset_stats.pkl')
        with open(stats_path, 'rb') as f:
            stats = pickle.load(f)
        return stats
    def load_policy(self) :
        """Load trained policy model"""
        policy =ACTPolicy(self.config['policy_config'])

        ckpt_path = os.path.join("/home/a/Downloads/mushroomproject/ros2_ws/src/hitbot_sim/hitbot_sim", 'policy_best.ckpt')
        policy.load_state_dict(torch.load(ckpt_path)) # type: ignore
        policy.cuda()
        policy.eval()
        #self.get_logger().info(f"Loaded policy from {ckpt_path},{policy.state_dict}")
        return policy
    def keyboard(self):
        print("hello hibot")
        running = True
        try:
            while running :
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        running = False
                keys =pygame.key.get_pressed()# input("input key")
                #self.get_logger().info(f"keys:{keys}")
                self.robot.get_scara_param()
                # Translation controls (WASD)
                if keys[pygame.K_w]:
                    #self.get_logger().info(f"A:{keys}")
                    self.robot.x =self.robot.x+20  # Forward
                if keys[pygame.K_s]:
                    self.robot.x =self.robot.x-20  # Backward
                if keys[pygame.K_a]:
                    self.robot.y =self.robot.y+20  # Left (strafe if holonomic)
                if keys[pygame.K_d]:
                    self.robot.y =self.robot.y-20  # Right (strafe if holonomic)
                if keys[pygame.K_z]:
                    self.robot.z =self.robot.z+20  # up (strafe if holonomic)
                if keys[pygame.K_x]:
                    self.robot.z =self.robot.z-30  # Down (strafe if holonomic)
                if keys[pygame.K_f]:
                    open=String()
                    open.data="400"
                    self.gripper_open_pub.publish(open)
                if keys[pygame.K_g]:
                    hold=String()
                    hold.data="300"
                    self.gripper_hold_pub.publish(hold)
                if keys[pygame.K_r]:
                    self.start_recording()

                # Quit on Q
                if keys[pygame.K_q]:
                    running = False
                if keys[pygame.K_t]:
                    self.stop_and_save()
                    pygame.time.delay(3)


                ret=self.robot.movel_xyz(self.robot.x,self.robot.y,self.robot.z,self.robot.r,30)
                self.robot.wait_stop()

                #self.get_logger().info(f"ret:{ret}")
        except ValueError as e:
            print("Error:", str(e))
        except RuntimeError as e:
            print("Error:", str(e))
        finally:
            pygame.quit()
            
    def run(self):
        print("hello hibot")
        #thread_key=threading.Thread(target=self.keyboard)
        #thread_key.start()


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
