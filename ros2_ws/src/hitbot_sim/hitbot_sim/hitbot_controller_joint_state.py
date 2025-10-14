import io
import math
import pickle
import subprocess
import sys
import os
import uuid
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
#import octomap
#from scipy.spatial import KDTree
#import fcl
from ikpy.chain import Chain
from ikpy.link import URDFLink
#import h5py
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
redis_server='172.23.248.33'

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

from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import PlanningScene, CollisionObject
from shape_msgs.msg import SolidPrimitive
import pymoveit2 
from sensor_msgs.msg import LaserScan
from moveit_msgs.msg import PlanningScene, CollisionObject
from geometry_msgs.msg import Pose

from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK


import numpy as np


class OldSolver:
    def __init__(self,link_lengths=np.array([0.325, 0.275, 0.254])):
        self.link_lengths = link_lengths
    #related angles
    def forward_kinematics_from_angles(self,theta):
        theta1, theta2, theta3 = theta
        x = self.link_lengths[0] * np.cos(theta1) + self.link_lengths[1] * np.cos(theta1 + theta2) + self.link_lengths[2] * np.cos(theta1 + theta2 + theta3)
        y = self.link_lengths[0] * np.sin(theta1) + self.link_lengths[1] * np.sin(theta1 + theta2) + self.link_lengths[2] * np.sin(theta1 + theta2 + theta3)
        return np.array([x, y])
    #absolute angles
    def check_forward_kinematics_from_angles(self,theta):
        theta1, theta2, theta3 = theta
        x = self.link_lengths[0] * np.cos(theta1) + self.link_lengths[1] * np.cos(  theta2) + self.link_lengths[2] * np.cos( theta3)
        y = self.link_lengths[0] * np.sin(theta1) + self.link_lengths[1] * np.sin( theta2) + self.link_lengths[2] * np.sin(theta3)
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
    def normalize_angle(self,deg):
        deg = ((deg + 180) % 360) - 180
        return round(deg, 1)

    def get_robot_angle_in_degree(self, target, current_angle=[0,0,0]):
        ans=self.get_angle_to_target(target, current_angle)
        if ans is None:  # Check if IK failed
            print("ERROR: No valid joint angles (IK failed)!")
            return [0.0, 0.0, 0.0]  # Default safe value (or raise an exception)
        theta1 = ans[0] * 180.0 / math.pi
        theta2 = ans[1] * 180.0 / math.pi
        theta3 = ans[2] * 180.0 / math.pi

    # 累积角度
        angles = [theta1, theta1 + theta2,theta1 + theta2 + theta3]
        normalized = [self.normalize_angle(a) for a in angles]
        return normalized

class Solver:
    def __init__(self, link_lengths=np.array([0.325, 0.275, 0.260])):
        self.link_lengths = link_lengths

    def forward_kinematics_from_angles(self, theta):
        t1, t2, t3 = theta
        x = (self.link_lengths[0] * np.cos(t1) +
             self.link_lengths[1] * np.cos(t1 + t2) +
             self.link_lengths[2] * np.cos(t1 + t2 + t3))
        y = (self.link_lengths[0] * np.sin(t1) +
             self.link_lengths[1] * np.sin(t1 + t2) +
             self.link_lengths[2] * np.sin(t1 + t2 + t3))
        return np.array([x, y])

    def jacobian(self, theta):
        r1, r2, r3 = self.link_lengths
        t1, t2, t3 = theta
        j11 = -r1*np.sin(t1) - r2*np.sin(t1+t2) - r3*np.sin(t1+t2+t3)
        j12 = -r2*np.sin(t1+t2) - r3*np.sin(t1+t2+t3)
        j13 = -r3*np.sin(t1+t2+t3)
        j21 = r1*np.cos(t1) + r2*np.cos(t1+t2) + r3*np.cos(t1+t2+t3)
        j22 = r2*np.cos(t1+t2) + r3*np.cos(t1+t2+t3)
        j23 = r3*np.cos(t1+t2+t3)
        return np.array([[j11, j12, j13],
                         [j21, j22, j23]])

    def inverse_kinematics(self, target, initial_guess, max_iter=100, tol=1e-5):
        theta = np.array(initial_guess, dtype=float)
        for _ in range(max_iter):
            current_pos = self.forward_kinematics_from_angles(theta)
            error = target - current_pos
            if np.linalg.norm(error) < tol:
                break
            J = self.jacobian(theta)
            dtheta = np.linalg.pinv(J) @ error
            theta += dtheta
        
        theta = (theta + np.pi) % (2 * np.pi) - np.pi
        return theta


class ServiceClient(Node):
    def __init__(self):
        super().__init__('service_client')
        #self.client = self.create_client(Trigger, 'close_gripper_service')  # Service type and name
        #self.open_client = self.create_client(Trigger, 'open_gripper_service')  # Service type and name
       
        # Wait for service to be available
        #while not self.client.wait_for_service(timeout_sec=1.0):
        #    self.get_logger().info('close service not available, waiting again...')
        #while not self.open_client.wait_for_service(timeout_sec=1.0):
        #    self.get_logger().info('open service not available, waiting again...')
       
        #self.request = Trigger.Request()
        #self.open_request = Trigger.Request()
   
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

        self.goal=[0,0]

        self.link_lengths=np.array([0.325, 0.275, 0.260])
        self.rt=RobotTransform(self.link_lengths)
        self.hitbot_x = 0
        self.hitbot_y = 0
        self.hitbot_z = 0
        self.hitbot_r = -0

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
        #self.adj_bounding_boxes_sub = self.create_subscription(String,"/yolox/adj_bounding_boxes",self.adj_bounding_boxes_callback, 10)
        self.xyz_sub = self.create_subscription(String,"/hitbot_end_xyz",self.hitbot_end_xyzr_callback,10)
        self.angle_sub = self.create_subscription(String,"/hitbot_end_angle",self.hitbot_end_angle_callback,10)
        self.gripper_adjust_sub = self.create_subscription(String,"/yolox/rpi5/adjust/xy_pixel",self.hitbot_gripper_adjust_callback,1)
        self.gripper_adj_done_sub = self.create_subscription(String,"/yolox/rpi5/adjust/done",self.hitbot_gripper_adjust_done_callback,1)
        self.gripper_adj_done_pub= self.create_publisher(String,'/yolox/rpi5/adjust/done',1)
        self.rpi5_adj_xy_pixel=[0,0]
        
        

        self.gripper_open_pub= self.create_publisher(String,'/yolox/gripper_open',10)
        self.gripper_hold_pub = self.create_publisher(String,'/yolox/gripper_hold',10)
        
        # Joint State Publisher
        self.joint_state_pub = self.create_publisher(JointState, "/joint_states", 10)

        # Timer to publish joint states at 50Hz (20ms)
        self.timer = self.create_timer(0.5, self.timer_cb)
        #self.timer = self.create_timer(10, self.thread_bounding_boxes_callback)

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


        
        # SCARA configuration
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
        self.gripper_joint = 'joint4'  # Assuming last joint controls gripper
        self.planned_once = False
        
        
        # ROS setup
        self.bridge = CvBridge()
   
        self.link_lengths=[np.float64(0.325), np.float64(0.275), np.float64(0.26)]


        self.sub_scan = self.create_subscription(LaserScan, "/scan", self.scan_callback, 10)
        self.pub_scene = self.create_publisher(PlanningScene, "/planning_scene", 10)
        self.get_logger().info("✅ 已启动: 将 /scan 转换为 3D 细小立柱障碍物，并查找最近障碍物")

        self.solver=OldSolver() #Solver()#

        self.obstacles=[]
        self.column_count=0

    def point_in_triangle(self, P, A, B, C):
        def cross(u, v): return u[0]*v[1] - u[1]*v[0]
        PA = (P[0]-A[0], P[1]-A[1])
        PB = (P[0]-B[0], P[1]-B[1])
        PC = (P[0]-C[0], P[1]-C[1])
        AB = (B[0]-A[0], B[1]-A[1])
        BC = (C[0]-B[0], C[1]-B[1])
        CA = (A[0]-C[0], A[1]-C[1])
        c1 = cross(AB, PA)
        c2 = cross(BC, PB)
        c3 = cross(CA, PC)
        return (c1 >= 0 and c2 >= 0 and c3 >= 0) or (c1 <= 0 and c2 <= 0 and c3 <= 0)
# ✅ 创建圆柱体障碍物
    def addobject(self,ps,x,y):
        ps.world.collision_objects.clear()

        co = CollisionObject()
        co.id = "pillar_" + str(uuid.uuid4())[:8]
        co.header.frame_id = "base_link"
        co.operation = CollisionObject.ADD
        cylinder = SolidPrimitive()
        cylinder.type = SolidPrimitive.CYLINDER
        cylinder.dimensions = [0.5, 0.02]  # 高0.5m 半径2cm

        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = 0.25  # 中心点高度
        pose.orientation.w = 1.0

        co.primitives.append(cylinder)
        co.primitive_poses.append(pose)
        ps.world.collision_objects.append(co)
        self.column_count += 1
    def scan_callback(self, msg: LaserScan):
        self.obstacles=[]
        self.column_count=0
        ps = PlanningScene()
        ps.is_diff = True

        co = CollisionObject()
        co.id = "all"                     # ID 可随意，只要唯一即可
        co.operation = CollisionObject.REMOVE
        ps.world.collision_objects.append(co)
        self.pub_scene.publish(ps)


        for i, r in enumerate(msg.ranges):
            if not (msg.range_min < r < msg.range_max):
                continue

            angle = msg.angle_min + i * msg.angle_increment
            x = r * math.cos(angle)
            y = r * math.sin(angle)
            P = (x, y)
            dist = math.sqrt(x**2 + y**2)
            self.obstacles.append(P)
            self.addobject(ps,x,y)


        # ✅ 发布所有立柱障碍物
        if self.column_count > 0:
            self.pub_scene.publish(ps)
            self.get_logger().info(f"📡 已发布 {self.column_count} 个立柱障碍物")

        time.sleep(5)
    def compute_waypoint(self, obstacle, current, target):
        """
        根据障碍点计算绕行点（自动判断左右绕行方向）
        """
        ox, oy = obstacle
        cx, cy = current
        tx, ty = target

        # 当前到目标的方向向量
        dx = tx - cx
        dy = ty - cy
        norm = math.sqrt(dx**2 + dy**2)
        if norm == 0:
            return current
        dx /= norm
        dy /= norm

        # 判断障碍在路径的哪一侧（通过叉积符号判断）
        cross_val = (ox - cx) * dy - (oy - cy) * dx
        side = 1 if cross_val < 0 else -1  # <0 左侧, >0 右侧

        # 计算垂直偏移方向
        perp = (-dy * side, dx * side)

        offset = 0.15  # 绕障偏移距离
        waypoint = (ox + perp[0]*offset, oy + perp[1]*offset)

        self.get_logger().info(
            f"🧭 绕行方向 {'左侧' if side > 0 else '右侧'} → waypoint=({waypoint[0]:.3f},{waypoint[1]:.3f})"
        )
        return waypoint

    def findpath(self, current, target):
        """
        在 (current, target, origin) 三角区域内寻找障碍，
        并按角度排序生成自然路径。
        """
        origin = (0.0, 0.0)
        valid_obs = []
        waypoints = []
        # --- 1️⃣ 筛选三角形内的障碍 ---
        for obs in self.obstacles:
            if self.point_in_triangle(obs, current, target, origin):
                valid_obs.append(obs)

        if not valid_obs:
            self.get_logger().info("✅ 三角区域内无障碍")
        #    waypoints.append(target)
        #    return waypoints

        # --- 2️⃣ 计算每个障碍相对于路径的角度 ---
        cx, cy = current
        tx, ty = target
        path_vec = (tx - cx, ty - cy)

        obs_angles = []
        for obs in valid_obs:
            ox, oy = obs
            obs_vec = (ox - cx, oy - cy)
            cross = path_vec[0]*obs_vec[1] - path_vec[1]*obs_vec[0]
            dot = path_vec[0]*obs_vec[0] + path_vec[1]*obs_vec[1]
            theta = math.atan2(cross, dot)   # [-pi, pi]
            dist = math.sqrt((ox - cx)**2 + (oy - cy)**2)
            obs_angles.append((obs, theta, dist))

        # --- 3️⃣ 按角度排序，再按距离排序（保证路径连续） ---
        obs_angles.sort(key=lambda x: (x[1], x[2]))

        # --- 4️⃣ 生成绕行点 ---

        for obs, theta, _ in obs_angles:
            waypoint = self.compute_waypoint(obs, current, target)
            waypoints.append(waypoint)

        self.get_logger().info(f"✅ 找到 {len(waypoints)} 个绕行点（按角度排序）")
        return waypoints


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
        x = self.link_lengths[0] * np.cos(theta1) + self.link_lengths[1] * np.cos(theta2) + self.link_lengths[2] * np.cos(theta3)
        y = self.link_lengths[0] * np.sin(theta1) + self.link_lengths[1] * np.sin(theta2) + self.link_lengths[2] * np.sin(theta3)
        #x = self.link_lengths[0] * np.cos(theta1) + self.link_lengths[1] * np.cos(theta1 + theta2) + self.link_lengths[2] * np.cos(theta1 + theta2 + theta3)
        #y = self.link_lengths[0] * np.sin(theta1) + self.link_lengths[1] * np.sin(theta1 + theta2) + self.link_lengths[2] * np.sin(theta1 + theta2 + theta3)
        #x=self.robot.x+self.link_lengths[2] * np.cos(self.robot.angle1*3.14/180 + self.robot.angle2*3.14/180 + self.robot.angle3*3.14/180)
        #y=self.robot.x+self.link_lengths[2] * np.sin(self.robot.angle1*3.14/180 + self.robot.angle2*3.14/180 + self.robot.angle3*3.14/180)
        return np.array([x, y])

    def plan_and_print_once(self,x,y,z):
        #if self.planned_once:
        #    return
        #self.planned_once = True

        theta_init = [0, 0, 0]
        
        waypoints=self.findpath((self.robot.x,self.robot.y),(x,y))
        if waypoints is None or len(waypoints)==0:
            waypoints=[]
            waypoints.append((x,y))
        for wp in waypoints:
            self.get_logger().info(f"➡️ 规划路径点: x={wp[0]:.3f}, y={wp[1]:.3f}")
            target = [wp[0], wp[1]]
            self.robot.get_scara_param()
            #current_angle=[ang1 *3.1415/180,ang2*3.1415/180,ang2*3.1415/180]
            #response=self.solver.get_robot_angle_in_degree(target)
            #response=self.solver.inverse_kinematics(target,current_angle)
            #response=self.solver.inverse_kinematics(target, theta_init)
            if 1:#response is not None:
                #joint_positions = response
                #forward_xy=self.solver.check_forward_kinematics_from_angles(np.deg2rad(joint_positions))
                #self.get_logger().info('✅ IK Computed Successfully!joint_pos={}'.format(joint_positions))
                #self.get_logger().info(' Target: x={:.3f},y={:.3f},z={:.3f}, Forward FK: x={:.3f},y={:.3f}'.format(x,y,z,forward_xy[0],forward_xy[1]))
                #if(y)<0:
                #    joint_positions[2]=joint_positions[2]+20
                #else:
                #    joint_positions[2]=joint_positions[2]-0
                #self.get_logger().info(' Move to: angle1={:.1f},angle2={:.1f},angle3={:.1f},z={:.1f}'.format(joint_positions[0],joint_positions[1],joint_positions[2],z))
                #joint
                #ret=self.robot.movej_angle(joint_positions[0],joint_positions[1],0,joint_positions[2]-0,30,1) 
                ret=self.robot.movej_xyz(target[0]*1000-260,target[1]*1000,0,0,30,1) 

            else:
                self.get_logger().error(f'❌ IK computation failed: code={response.error_code.val}')
            #time.sleep(0.5)
        self.robot.wait_stop()
        waypoints=[]


     
    def thread_bounding_boxes_callback(self):
        self.bounding_boxes_callback()
            #threading.Thread(target=self.bounding_boxes_callback, args=(msg,),daemon=True).start()
    def bounding_boxes_callback(self, msg):
        #r.set("gripper_flag", "ready")
        if r.get("mode")=="camera_ready":
            mushroom_xyz=msg.data
            mushroom_xyz=msg.data.split(",");
            #mushroom_xyz=r.get("mushroom_xyz")
            if mushroom_xyz is None :
                self.get_logger().info(f"no mushroom_xyz")
                return
            goal=[int(float(mushroom_xyz[2].strip())),0-int(float(mushroom_xyz[0].strip()))]
            self.get_logger().info(f"get mushroom_xyz:{mushroom_xyz}")
            #self.goal=[int(float(mushroom_xyz[2].strip()))/1000,0-int(float(mushroom_xyz[0].strip()))/1000]
            #self.get_logger().info(f"target get goal:{self.goal}")        

            #ret=self.robot.movej_angle(angles[0],angles[1],0,angles[2]-0,100,1) 
            #ret=self.robot.movej_xyz(goal[0],goal[1],0,-0,80,1)
            self.robot.get_scara_param()
            self.plan_and_print_once(x=goal[0]/1000,y=goal[1]/1000,z=0.1)
            self.get_logger().info(f"bounding_boxes_callback : goal={goal},")
            self.robot.wait_stop()
            time.sleep(2)
            #if ret<2:
            #    r.set("mode","adjust_ready")
            #else:
            #	r.set("mode","camera_ready")    
        r.set("gripper_flag", "done")
        r.set("mushroom_xyz","")    

    def adj_bounding_boxes_callback(self, msg):
        if r.get("mode") == "adjust_ready":
            mushroom_xyz = msg.data
            mushroom_xyz = msg.data.split(",")
            goal = [int(float(mushroom_xyz[0].strip())), int(float(mushroom_xyz[1].strip())), int(float(mushroom_xyz[2].strip()))]
            self.robot.get_scara_param()
            if(abs(self.robot.x - goal[2])>80 or abs(self.robot.y - goal[0])>50):
                ret = self.robot.movej_xyz(self.robot.x - goal[2], self.robot.y - goal[0], self.robot.z, -0, 80, 1)
                self.get_logger().info(f"adj bounding_boxes_callback ->adjust :{goal},x_offset:{goal[2]},y_offset:{goal[0]},ret :{ret}")
            else:
                self.robot.wait_stop()
                r.set("mode", "adjust_done")
            if 0:  # ret<2:
                r.set("mode", "adjust_done")
                response = self.client_node.open_send_request()
                # if response is not None:
                self.robot.get_scara_param()
                self.robot.wait_stop()
                ret = self.robot.movej_xyz(self.robot.x, self.robot.y, self.robot.z - 170, self.robot.r, 80, 1)
                self.robot.wait_stop()
                self.get_logger().info(f"move in -z, ret :{ret}")
                response = self.client_node.send_request()  # close request
                time.sleep(3)
                self.robot.get_scara_param()
                self.robot.wait_stop()
                self.client_node.get_logger().info(f'open 2 for {self.robot.z}')
                for i in range(3):
                    ret = self.robot.movej_xyz(self.robot.x, self.robot.y, self.robot.z, self.robot.r - 3, 30, 1)
                    self.robot.wait_stop()
                    self.robot.get_scara_param()
                    self.robot.wait_stop()
                    ret = self.robot.movej_xyz(self.robot.x, self.robot.y, self.robot.z, self.robot.r + 3, 30, 1)
                    self.robot.wait_stop()
                    time.sleep(0.5)
                # if response is not None:
                self.robot.get_scara_param()
                ret = self.robot.movej_xyz(self.robot.x, self.robot.y, 0, self.robot.r, 80, 1)
                self.robot.wait_stop()
                time.sleep(1)
                ret = self.robot.movej_xyz(0, 400, 0, +20, 50, 1)
                self.robot.wait_stop()
                response = self.client_node.open_send_request()
            # else:
            #     r.set("mode","camera_ready")
            r.set("mode", "camera_ready")

        time.sleep(2)
        r.set("gripper_flag", "done")

    def hitbot_gripper_adjust_done_callback(self, msg):
        self.get_logger().info(f"hitbot_gripper_adjust_done_callback:{msg}")
        if  r.get("mode")=="adjust_done":
            response = self.client_node.open_send_request()
            if response is not None:
                self.robot.get_scara_param()
                self.robot.wait_stop()
                ret=self.robot.movej_xyz(self.robot.x,self.robot.y,self.robot.z-0,self.robot.r,80,1)
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
                #ret=self.robot.movej_angle(angles[0],angles[1],0,angles[2]-0,50,1)
                #adj_goal=[(self.robot.x+x_0/30),(self.robot.y+y_0/30)]
                adj_goal=[(self.robot.x-int(xy[1])/10),(self.robot.y-int(xy[0])/10)] 
                
                ret=self.robot.movej_xyz(adj_goal[0],adj_goal[1],self.robot.z,-0,30,1)
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
        #self.get_logger().info("joint_state_cb:{}".format(msg.position))  
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
    
    def timer_cb(self):
        self.publish_joint_states()
        #threading.Thread(target=self.publish_joint_states, daemon=True).start()

    def publish_joint_states(self):
        # Get real joint positions from HitBot API (Replace this with actual API calls)
        self.robot.get_scara_param()
        joint_positions = [self.robot.z,self.robot.angle1*3.14/180,self.robot.angle2*3.14/180,(self.robot.r+20)*3.14/180]

        # Create JointState message
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = self.joint_names
        joint_state_msg.position = joint_positions
        # Publish the joint states
        self.get_logger().info(f"publish_joint_states :joint_positions={joint_positions}")
        self.joint_state_pub.publish(joint_state_msg)

        self.hitbot_x=self.robot.x/1000
        self.hitbot_y=self.robot.y/1000
        self.hitbot_z=self.robot.z/1000

        self.publish_hitbot_x(str(int(self.robot.x)))
        self.publish_hitbot_y(str(int(self.robot.y)))
        self.publish_hitbot_z(str(int(self.robot.z)))
        #self.get_logger().info(f"publish_joint_states z={str(int(self.robot.z))}")
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
                self.robot.new_movej_angle(waypoints[1]*180/3.14, waypoints[2]*180/3.14, waypoints[0], (waypoints[3]*180/3.14)-0.0, 50, 1)
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
        ret=self.robot.movel_xyz(600,0,0,-0,50)
    	#ret=self.robot.new_movej_xyz_lr(hi.x-100,hi.y,0,-0,20,0,1)
        #ret=self.robot.movej_angle(0,30,0,0,20,0)
        self.robot.wait_stop()
        r.set("mode","camera_ready") 
        print('Robot I/O output initialized.')
        for i in range(12):
            self.robot.set_digital_out(i, False)
        time.sleep(1)
        print('Robot initialized.')
    

    def run(self):
        print("hello hibot")
        executor = MultiThreadedExecutor(num_threads=10)  # 开4个线程并行处理回调
        executor.add_node(self)
        #executor.add_node(self.compute_ik_node)

        try:
            executor.spin()
        except KeyboardInterrupt:
            self.get_logger().info("⚠️ Ctrl+C 停止")
        finally:
            self.destroy_node()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)

    hitbot_controller = HitbotController()
    hitbot_controller.run()
    

if __name__ == "__main__":
    main()
