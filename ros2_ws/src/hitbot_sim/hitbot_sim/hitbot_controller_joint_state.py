import io
import json
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
from rclpy.callback_groups import ReentrantCallbackGroup
from nav_msgs.msg import Path
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

from tqdm import tqdm
from scipy.spatial import KDTree
import numpy as np
from scipy.interpolate import splprep, splev
import numpy as np
L1, L2, L3 = 0.325, 0.275, 0.26
class RRTAngle:
    def __init__(self, start_pos, obstacles=[], tolerance=0.1, step_size=0.15):
        self.length = [0.325, 0.275,0.26]
        self.workspace_size = sum(self.length)
        self.start_node = np.array(start_pos)
        self.tolerance = tolerance
        self.step_size = step_size
        self.tree = {tuple(self.start_node): None}
        self.goal_node = None
        self.obstacles = obstacles
        self.forward_kinematics_cache = {}  # Cache for forward kinematics results
        self.kd_tree = KDTree([self.start_node])  # KDTree for nearest neighbor search
        self.solver=OldSolver()

    def set_start_pos(self,  params):
        self.start_node = np.array(params)
        self.tree = {tuple(self.start_node): None}
        self.kd_tree = KDTree([self.start_node])  # KDTree for nearest neighbor search
    def add_obstacle(self, shape, params):
        self.obstacles.append((shape, params))

    def forward_kinematics(self, angles):
        angles_tuple = tuple(angles)  # Use tuple to cache based on angles
        if angles_tuple in self.forward_kinematics_cache:
            return self.forward_kinematics_cache[angles_tuple]

        x0, y0 = 0, 0
        x1 = x0 + self.length[0] * np.cos(angles[0])
        y1 = y0 + self.length[0] * np.sin(angles[0])
        x2 = x1 + self.length[1] * np.cos(angles[0] + angles[1])
        y2 = y1 + self.length[1] * np.sin(angles[0] + angles[1])
        x3 = x2 + self.length[2] * np.cos(angles[0] + angles[1]+angles[2])
        y3 = y2 + self.length[2] * np.sin(angles[0] + angles[1]+ angles[2])

        result = np.array([[x0, x1, x2,x3], [y0, y1, y2,y3]], dtype=np.float32)
        self.forward_kinematics_cache[angles_tuple] = result  # Cache the result
        return result
    def sample_position_new(self, goal_ws=None, last_pos=None, goal_bias=0.3, max_attempts=20):
        """
        goal_ws: 目标点 (x, y)
        last_pos: 上一个节点末端坐标，用于局部方向判断
        """
        # 有目标且触发目标偏置时
        if goal_ws is not None and np.random.rand() < goal_bias:
            for _ in range(max_attempts):
                sample = [
                    np.random.uniform(-np.pi/2, np.pi/2),
                    np.random.uniform(-3*np.pi/4, 3*np.pi/4),
                    np.random.uniform(-3*np.pi/4, 3*np.pi/4)
                ]
                xs, ys = self.forward_kinematics(sample)
                end_effector = np.array([xs[-1], ys[-1]])

                # 计算局部方向向量（上一个点指向目标）
                if last_pos is None:
                    last_pos = np.array([0.8, 0.0])  # 如果是起点，默认用原点

                vec_goal = np.array(goal_ws) - np.array(last_pos)
                vec_end = end_effector - np.array(last_pos)

                # 判断夹角
                cos_theta = np.dot(vec_goal, vec_end) / (
                    np.linalg.norm(vec_goal) * np.linalg.norm(vec_end) + 1e-6
                )

                # 方向一致且距离适中时，接受采样
                if cos_theta > 0.4 or np.linalg.norm(end_effector - goal_ws) < self.workspace_size *0.3:
                    return sample

        # 默认随机采样
        return [
            np.random.uniform(-np.pi/2, np.pi/2),
            np.random.uniform(-3*np.pi/4, 3*np.pi/4),
            np.random.uniform(-3*np.pi/4, 3*np.pi/4)
        ]


    def sample_position(self, goal_ws=None, goal_bias=0.2):
        if goal_ws is not None and np.random.rand() < goal_bias:
            # Try goal-biased sampling
            for _ in range(10):  # Try multiple times to get a good sample
                sample = [np.random.uniform(-np.pi/2, np.pi/2),
                        np.random.uniform(-3*np.pi/4, 3*np.pi/4),
                        np.random.uniform(-3*np.pi/4, 3*np.pi/4)]
                xs, ys = self.forward_kinematics(sample)
                end_effector = np.array([xs[-1], ys[-1]])
                if np.linalg.norm(end_effector - goal_ws) < self.workspace_size * 0.2:
                    return sample
        # Otherwise, uniform sampling
        return [np.random.uniform(-np.pi/2, np.pi/2),
                np.random.uniform(-3*np.pi/4, 3*np.pi/4),
                np.random.uniform(-3*np.pi/4, 3*np.pi/4)]

    def is_in_obstacle(self, angles):
        xs, ys = self.forward_kinematics(angles)
        for x1, y1, x2, y2 in zip(xs, ys, xs[1:], ys[1:]):
            for shape, params in self.obstacles:
                if shape == 'circle':
                    cx, cy, r = params
                    if self._line_circle_collision(x1, y1, x2, y2, cx, cy, r):
                        return True
                elif shape == 'rectangle':
                    x, y, w, h = params
                    if self._line_rect_collision(x1, y1, x2, y2, x, y, w, h):
                        return True
        return False

    def _line_circle_collision(self, x1, y1, x2, y2, cx, cy, r):
        d = np.array([x2 - x1, y2 - y1])
        f = np.array([x1 - cx, y1 - cy])
        a = np.dot(d, d)
        b = 2 * np.dot(f, d)
        c = np.dot(f, f) - r * r
        discriminant = b * b - 4 * a * c
        if discriminant < 0:
            return False
        discriminant = np.sqrt(discriminant)
        t1 = (-b - discriminant) / (2 * a)
        t2 = (-b + discriminant) / (2 * a)
        return (0 <= t1 <= 1) or (0 <= t2 <= 1)

    def _line_rect_collision(self, x1, y1, x2, y2, rx, ry, rw, rh):
        for t in np.linspace(0, 1, 20):
            xt = x1 + t * (x2 - x1)
            yt = y1 + t * (y2 - y1)
            if rx <= xt <= rx + rw and ry <= yt <= ry + rh:
                return True
        return False

    def nearest_node(self, sample):
        # Use KDTree for efficient nearest neighbor search
        dist, idx = self.kd_tree.query([sample], k=1)
        nearest_node = tuple(self.kd_tree.data[idx][0])  # Get the nearest node tuple
        return nearest_node

    def steer(self, from_node, to_node):
        direction = np.array(to_node) - np.array(from_node)
        distance = np.linalg.norm(direction)
        if distance <= self.step_size:
            return np.array(to_node)
        direction /= distance
        return np.array(from_node) + self.step_size * direction

    def distance_in_workspace(self, angles, goal_ws):
        xs, ys = self.forward_kinematics(angles)
        end_effector = np.array([xs[-1], ys[-1]])
        distance = np.linalg.norm(end_effector - goal_ws)
        return distance

    def build_tree(self, goal_ws, max_iterations=10000):
        goal_ws = np.array(goal_ws, dtype=np.float32)

        for i in tqdm(range(max_iterations)):
            sample = self.sample_position()
            nearest = self.nearest_node(sample)


            # 朝目标方向采样sample_position_new
            #xs, ys = self.forward_kinematics(nearest)
            #last_pos = np.array([xs[-1], ys[-1]])
            #sample = self.sample_position(goal_ws, last_pos=last_pos)

            new_node = self.steer(nearest, sample)
            new_node_tuple = tuple(new_node)

            distance = self.distance_in_workspace(new_node, goal_ws)

            if new_node_tuple in self.tree:
                continue
            if self.is_in_obstacle(new_node):
                continue

            self.tree[new_node_tuple] = tuple(nearest)
            self.kd_tree = KDTree(list(self.tree.keys()))  # Rebuild the KDTree after adding new node

            if distance < self.tolerance:
                self.goal_node = new_node_tuple
                print(f"Goal reached in {i} iterations!")
                smooth_curve,smooth_joint_path=self.reconstruct_path()
                return smooth_curve,smooth_joint_path

        print("Goal not reached within iteration limit.")
        return None

    def reconstruct_path(self):
        path = []
        node = self.goal_node
        while node is not None:
            path.append(node)
            node = self.tree[node]
        path= path[::-1]
        print(f"Raw path length: {len(path)}")
        smooth_curve,smooth_joint_path = self.spline_smooth(path)
        average_curve, average_joint_path = self.moving_average(smooth_curve, smooth_joint_path)
        return average_curve, average_joint_path




    def moving_average(self, path, smooth_joint_path, window=8):
        """
        对笛卡尔路径和平行关节路径一起做移动平均
        返回:
            smoothed_xy: 平滑后的笛卡尔路径
            smoothed_joint: 平滑后的关节角路径
        """
        smoothed_xy = []
        smoothed_joint = []

        for i in range(len(path)):
            start = max(0, i - window)
            end = min(len(path), i + window)

            # --- 笛卡尔路径 ---
            xs = [p[0] for p in path[start:end]]
            ys = [p[1] for p in path[start:end]]
            smoothed_xy.append((sum(xs)/len(xs), sum(ys)/len(ys)))

            # --- 关节路径（多维平均）---
            n_dim = len(smooth_joint_path[0])
            avg_joint = tuple(
                sum(q[d] for q in smooth_joint_path[start:end]) / (end - start)
                for d in range(n_dim)
            )
            smoothed_joint.append(avg_joint)

        return smoothed_xy, smoothed_joint


    def spline_smooth(self, path, smooth_factor=0.0):
        """
        对关节角路径做样条平滑
        1️⃣ 用 FK 把每个关节角转为末端坐标 (x, y)
        2️⃣ 对笛卡尔路径平滑
        3️⃣ 返回平滑后的末端坐标序列
        """
        if len(path) < 3:
            return path

        # ① 将每个 joint 角度转为笛卡尔坐标
        xy_path = []
        for q in path:
            pts = self.fk(q)   # [[0,x1,x2,x3],[0,y1,y2,y3]]
            x, y = pts[0, -1], pts[1, -1]  # 末端点
            xy_path.append((x, y))

        x, y = zip(*xy_path)
        tck, _ = splprep([x, y], s=smooth_factor)
        u = np.linspace(0, 1, len(path))
        x_new, y_new = splev(u, tck)
        smooth_xy=list(zip(x_new, y_new))
        # ③ 尝试找到平滑后点对应的原 path 插值（用最近点近似）
        smooth_path = []
        for p in smooth_xy:
            # 找到最近的原始点对应的关节角
            idx = np.argmin([np.hypot(p[0]-px, p[1]-py) for px, py in xy_path])
            smooth_path.append(path[idx])

        return smooth_xy, smooth_path


    def smooth_path(self, path, max_trials=200):
        """Shortcut 平滑路径"""
        if len(path) <= 2:
            return path

        smoothed = path.copy()
        for _ in range(max_trials):
            if len(smoothed) < 3:
                break

            # 随机选取两点
            i, j = sorted(random.sample(range(len(smoothed)), 2))
            if j - i < 2:
                continue

            p1, p2 = smoothed[i], smoothed[j]

            # 如果这两点之间无障碍 → 删除中间节点
            if self.path_collision_free(p1, p2):
                smoothed = smoothed[:i+1] + smoothed[j:]

        return smoothed

    def path_collision_free(self, p1, p2, steps=3):
        """沿路径插补并检测机械臂连杆是否碰撞"""
        for i in range(steps + 1):
            print(f"i:{i},p1={p1},p2={p2} ")
            t = i / steps
            x = p1[0] + (p2[0] - p1[0]) * t
            y = p1[1] + (p2[1] - p1[1]) * t
            ik_sol = self.solver.get_robot_angle_in_rad([x, y],current_angle=[0,0,0])
            if ik_sol is None:
                return False
            if 1:#self.scara_in_collision(ik_sol):
                return False
        return True
    def point_to_segment_distance(self,p, a, b):
        px, py = p
        ax, ay = a
        bx, by = b
        dx, dy = bx - ax, by - ay
        if dx == dy == 0:
            return math.hypot(px-ax, py-ay)
        t = ((px-ax)*dx + (py-ay)*dy) / (dx*dx + dy*dy)
        t = max(0, min(1, t))
        cx, cy = ax + t*dx, ay + t*dy
        return math.hypot(px-cx, py-cy)

    def fk(self,q):
        θ1, θ2, θ3 = q
        x1 = L1 * math.cos(θ1)
        y1 = L1 * math.sin(θ1)
        x2 = x1 + L2 * math.cos(θ1 + θ2)
        y2 = y1 + L2 * math.sin(θ1 + θ2)
        x3 = x2 + L3 * math.cos(θ3)  # θ3 是绝对角度
        y3 = y2 + L3 * math.sin(θ3)
        return np.array([[0, x1, x2, x3], [0, y1, y2, y3]])

    def scara_in_collision(self, q):
        pts = self.fk(q)  # [[x0,x1,x2,x3],[y0,y1,y2,y3]]

        # ✅ 遍历所有类型的障碍物
        for shape, obs in self.obstacles:
            if 1:#for obs in obs_list:
                if shape == 'circle':
                    ox, oy, r = obs
                    for i in range(3):
                        a = (pts[0, i], pts[1, i])
                        b = (pts[0, i+1], pts[1, i+1])
                        d = self.point_to_segment_distance((ox, oy), a, b)
                        if d < r:
                            return True

                elif shape == 'square':
                    cx, cy, size = obs
                    for i in range(3):
                        a = (pts[0, i], pts[1, i])
                        b = (pts[0, i+1], pts[1, i+1])
                        if self.point_in_square((cx, cy), a, b, size/2):
                            return True
        return False
    def normalize_angle(self,deg):
        deg = ((deg + 180) % 360) - 180
        return round(deg, 1)
    def get_robot_angle_in_degree(self, ans):
        if ans is None:  # Check if IK failed
            print("ERROR: No valid joint angles (IK failed)!")
            return [0.0, 0.0, 0.0]  # Default safe value (or raise an exception)
        print(f" IK angles :{ans}")
        theta1 = ans[0] * 180.0 / math.pi
        theta2 = ans[1] * 180.0 / math.pi
        theta3 = ans[2] * 180.0 / math.pi
    # 累积角度
        angles = [theta1, theta2,theta1 + theta2 + theta3]
        normalized = [self.normalize_angle(a) for a in angles]
        return normalized

class OldSolver:
    def __init__(self,link_lengths=np.array([0.325, 0.275, 0.26])):
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
        x = self.link_lengths[0] * np.cos(theta1) + self.link_lengths[1] * np.cos(theta2) + self.link_lengths[2] * np.cos( theta3)
        y = self.link_lengths[0] * np.sin(theta1) + self.link_lengths[1] * np.sin( theta2) + self.link_lengths[2] * np.sin(theta3)
        return np.array([x, y])
    # for robot arm, theta1 theta2 are relative angles and theta3 is absolute angles
    def check_forward_kinematics_from_gripper_position(self,theta):
        theta1, theta2, theta3 = theta
        x = self.link_lengths[0] * np.cos(theta1) + self.link_lengths[1] * np.cos(theta1+theta2) + self.link_lengths[2] * np.cos(theta3)
        y = self.link_lengths[0] * np.sin(theta1) + self.link_lengths[1] * np.sin(theta1+theta2) + self.link_lengths[2] * np.sin(theta3)
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
        initial_guesses=[[np.random.uniform(*limits[i]) for i in range(3)] for i in range(0,200)]
        ik_success=[]
        for guess in initial_guesses:
            guess[2]=30*3.1416/180
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
        print(f" IK angles :{ans}")
        if ans is None:  # Check if IK failed
            print("ERROR: No valid joint angles (IK failed)!")
            return [0.0, 0.0, 0.0]  # Default safe value (or raise an exception)
        if abs(self.normalize_angle(ans[2])*180/math.pi)>150:
            print("ERROR: potential collision for gripper to link (IK failed)!")
            return [0.0, 0.0, 0.0]
        theta1 = ans[0] * 180.0 / math.pi
        theta2 = ans[1] * 180.0 / math.pi
        theta3 = ans[2] * 180.0 / math.pi


    # 累积角度
        angles = [theta1, theta2,theta1 + theta2 + theta3]
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
        cb_group = ReentrantCallbackGroup()
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


        self.bounding_boxes_sub = self.create_subscription(String,"/d435/yolox/bounding_boxes",self.bounding_boxes_callback, 2)
        self.adj_bounding_boxes_sub = self.create_subscription(String,"/d405/yolox/adj_bounding_boxes",self.adj_bounding_boxes_callback, 10)
        self.xyz_sub = self.create_subscription(String,"/hitbot_end_xyz",self.hitbot_end_xyzr_callback,10)
        self.angle_sub = self.create_subscription(String,"/hitbot_end_angle",self.hitbot_end_angle_callback,10)
        self.rpi5_adj_xy_pixel=[0,0]
        
        self.gripper_open_pub= self.create_publisher(String,'/yolox/gripper_open',10)
        self.gripper_hold_pub = self.create_publisher(String,'/yolox/gripper_hold',10)
        
        # Joint State Publisher
        self.joint_state_pub = self.create_publisher(JointState, "/hitbot/joint_states", 10)

        # Timer to publish joint states at 50Hz (20ms)
        self.timer = self.create_timer(0.1, self.timer_cb)
        #self.timer = self.create_timer(0.5, self.check_collision_callback,callback_group=cb_group)

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
        #self.path_publisher = self.create_publisher(Marker, 'rrt_path', 10)
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


        #self.sub_scan = self.create_subscription(LaserScan, "/scan", self.scan_callback, 10)
        self.pub_scene = self.create_publisher(PlanningScene, "/planning_scene", 10)
        self.pub_path = self.create_publisher(Path, '/rrt_path', 10)
        #self.get_logger().info("✅ 已启动: 将 /scan 转换为 3D 细小立柱障碍物，并查找最近障碍物")

        self.solver=OldSolver() #Solver()#

        self.obstacles=[]
        self.column_count=0

        # 参数
        self.step_size = 0.1
        self.max_iter = 500
        self.goal_tolerance = 0.1
        self.rrt_planner = RRTAngle(start_pos=[0.0, 0.0, 0.0], obstacles=[], tolerance=self.goal_tolerance, step_size=self.step_size)
        self.move_robo_angles=[]
  

    def find_nearest_obstacle(self, robot_pos=(0.0, 0.0)):
        self.robot.get_scara_param()
        self.robot.wait_stop
        current_angle=[self.robot.angle1,self.robot.angle2,self.robot.r]
        xy=self.solver.check_forward_kinematics_from_gripper_position(np.deg2rad(current_angle))
        robot_pos=[xy[0],xy[1]]
        """找到距离机器人最近的障碍物"""
        redis_obstacles = json.loads(r.get('obstacles'))
        if not redis_obstacles:
            return None, float('inf')

        points = np.array(redis_obstacles)
        dx = points[:, 0] - robot_pos[0]
        dy = points[:, 1] - robot_pos[1]
        dist = np.sqrt(dx**2 + dy**2)
        min_idx = np.argmin(dist)
        nearest = points[min_idx]
        min_dist = dist[min_idx]
        return tuple(nearest), min_dist

    def point_in_rotated_square(self,obs, current, target, half_width=0.1):
        """
        判断点 obs 是否在沿 current→target 方向的矩形路径内
        参数：
            obs: (x, y)
            current: (x, y)
            target: (x, y)
            half_width: 矩形宽度的一半（即路径安全半径）
        """
        p = np.array(obs)
        a = np.array(current)
        b = np.array(target)

        ab = b - a
        length = np.linalg.norm(ab)
        if length == 0:
            return False

        # 单位方向向量
        ab_unit = ab / length

        # obs 相对 current 的向量
        ap = p - a

        # 在路径方向的投影长度
        proj_len = np.dot(ap, ab_unit)
        # 垂直方向距离
        perp_dist = np.cross(ab_unit, ap)

        # ✅ 判断是否在矩形范围内
        in_length_range = (0 <= proj_len <= length)
        in_width_range = (abs(perp_dist) <= half_width)
        return in_length_range and in_width_range

    def point_in_square(self,obs, current, target, margin=0.0):
        """
        判断点 obs 是否在 current 与 target 形成的方框内
        参数：
            obs: (x, y)  — 障碍点
            current: (x, y) — 起点
            target: (x, y)  — 终点
            margin: float — 外扩边界（例如机器人宽度）
        返回：
            True / False
        """
        #target = [0.0,0.8]
        x_min = min(current[0], target[0]) - margin
        x_max = max(current[0], target[0]) + margin
        y_min = min(current[1], target[1]) - margin
        y_max = max(current[1], target[1]) + margin
        self.get_logger().info(f'current:{current},target:{target},obs:{obs}')

        ox, oy = obs
        return (x_min <= ox <= x_max) and (y_min <= oy <= y_max)
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


    def circle_center(self,A, B, C):
        x1, y1 = A
        x2, y2 = B
        x3, y3 = C
        temp = 2 * (x1*(y2 - y3) + x2*(y3 - y1) + x3*(y1 - y2))
        if abs(temp) < 1e-9:
            raise ValueError("A, B, C are collinear, no circle can be formed")
        cx = ((x1**2 + y1**2)*(y2 - y3) + (x2**2 + y2**2)*(y3 - y1) + (x3**2 + y3**2)*(y1 - y2)) / temp
        cy = ((x1**2 + y1**2)*(x3 - x2) + (x2**2 + y2**2)*(x1 - x3) + (x3**2 + y3**2)*(x2 - x1)) / temp
        return (cx, cy)

    def point_in_arc_old(self,P, A, B, C, tol=1e-2):
        cx, cy = self.circle_center(A, B, C)
        r = math.hypot(A[0]-cx, A[1]-cy)
        
        # 向量
        def angle(pt):
            return math.atan2(pt[1]-cy, pt[0]-cx)
        
        angA = angle(A)
        angB = angle(B)
        angC = angle(C)
        angP = angle(P)
        
        # 判断弧的方向 (顺时针或逆时针)
        def normalize(a):
            a = a % (2*math.pi)
            return a
        
        angA, angB, angC, angP = map(normalize, [angA, angB, angC, angP])
        
        # 通过B判断弧方向：如果B在A->C的顺时针方向，则弧为顺时针
        def is_between(a, b, c):
            # 判断 b 是否在 a->c 的逆时针方向上
            if a <= c:
                return a <= b <= c
            else:
                return b >= a or b <= c
        
        arc_is_ccw = is_between(angA, angB, angC)
        in_arc = is_between(angA, angP, angC) if arc_is_ccw else not is_between(angA, angP, angC)
        
        # 判断半径（是否在圆上或圆内）
        rp = math.hypot(P[0]-cx, P[1]-cy)
        in_radius = abs(rp - r) < tol or rp < r
        
        return in_arc and in_radius

    def point_in_arc(self, obs, current, target, max_radius=0.9):
        """
        判断点 obs 是否位于以原点为圆心、由 origin→current 和 origin→target 两射线夹成的扇形内，
        且 obs 到原点的半径 ≤ max_radius。

        参数:
            obs: (x, y)         # 障碍点坐标
            current: (x, y)     # 起始方向点
            target: (x, y)      # 结束方向点
            max_radius: float   # 半径上限（例如 0.9）

        返回:
            True / False
        """
        origin = np.array([0.0, 0.0])
        obs = np.array(obs, dtype=float)
        current = np.array(current, dtype=float)
        target = np.array(target, dtype=float)

        # 计算三条射线的极角
        angle_current = math.atan2(current[1], current[0])
        angle_target  = math.atan2(target[1], target[0])
        angle_obs     = math.atan2(obs[1], obs[0])

        # 统一角度到 [0, 2π)
        def norm(a):
            return (a + 2 * math.pi) % (2 * math.pi)

        a1 = norm(angle_current)
        a2 = norm(angle_target)
        a3 = norm(angle_obs)

        # 判断角度是否在范围内（考虑跨越0的情况）
        if a1 <= a2:
            in_angle = (a1 <= a3 <= a2)
        else:
            in_angle = (a3 >= a1) or (a3 <= a2)

        # 判断半径是否小于 max_radius
        dist = np.linalg.norm(obs - origin)
        in_radius = dist <= max_radius

        return in_angle and in_radius
    def point_in_arc_link(self, obs, current, target, link_length=0.254, half_width=0.05):
        """
        判断障碍点 obs 是否位于以第二杆末端 current 为起点、
        指向 target 方向、长度为 link_length 的矩形区域内。

        参数:
            obs: (x, y)         # 障碍点坐标
            current: (x, y)     # 第二杆末端坐标
            target: (x, y)      # 末端目标坐标
            link_length: float  # 第三连杆长度
            half_width: float   # 连杆宽度的一半（安全边距）
        返回:
            True / False
        """

        # numpy 化
        p = np.array(obs, dtype=float)
        a = np.array(current, dtype=float)
        b = np.array(target, dtype=float)

        # 连杆方向向量
        ab = b - a
        norm_ab = np.linalg.norm(ab)
        if norm_ab < 1e-6:
            return False

        # 单位方向
        ab_unit = ab / norm_ab

        # “弧段”的末端点（长度为 link_length）
        arc_end = a + ab_unit * link_length

        # 现在我们要判断 obs 是否在 (a→arc_end) 为长、宽 2×half_width 的矩形内
        ap = p - a

        # 在路径方向的投影长度
        proj_len = np.dot(ap, ab_unit)
        # 垂直偏移（2D 叉积表示）
        perp_dist = np.cross(ab_unit, ap)

        in_length_range = (0.0 <= proj_len <= link_length)
        in_width_range  = (abs(perp_dist) <= half_width)

        return in_length_range and in_width_range


# ✅ 创建圆柱体障碍物
    def addobject(self,ps,x,y):
 

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
        ps = PlanningScene()
        ps.is_diff = True
        #ps.world.collision_objects.clear()
        self.column_count = 0
        points = []
        self.get_logger().warn("⚠️ scan callback triggered")

        angle = msg.angle_min
        for i, radius in enumerate(msg.ranges):
            if not (msg.range_min < radius < msg.range_max):
                continue

            angle = msg.angle_min + i * msg.angle_increment
            x = radius * math.cos(angle)
            y = radius * math.sin(angle)
            P = (x, y)
            dist = math.sqrt(x**2 + y**2)
            if 0.4 < dist < 1.2:
                #self.addobject(ps,x,y)
                points.append(P)

        if not points:
            self.get_logger().warn("⚠️ 本帧未检测到有效障碍点")
            return

        # 2️⃣ 按扫描角度顺序聚类
        clusters = []
        cluster = [points[0]]
        cluster_thresh = 0.05  # 相邻点间距小于 5cm 视为同一簇

        for i in range(1, len(points)):
            prev = points[i - 1]
            curr = points[i]
            d = math.hypot(curr[0] - prev[0], curr[1] - prev[1])
            if d < cluster_thresh:
                cluster.append(curr)
            else:
                clusters.append(cluster)
                cluster = [curr]
        clusters.append(cluster)

        self.get_logger().info(f"🎯 聚类结果: {len(points)} 点 → {len(clusters)} 个簇")

        # 3️⃣ 每个簇取中心点作为一个障碍物
        for cluster in clusters:
            ox = (-1.0)*np.mean([p[0] for p in cluster])
            oy = (-1.0)*np.mean([p[1] for p in cluster])
            self.obstacles.append((ox, oy))


        self.get_logger().info(f"检测到 {len(self.obstacles)} 个障碍物")
        for i, (cx, cy) in enumerate(self.obstacles):
            self.addobject(ps, cx, cy)
            self.get_logger().info(f"障碍物 {i+1}: x={cx:.2f}, y={cy:.2f}")

        # ✅ 发布所有立柱到规划场景
        if self.column_count > 0:
            self.pub_scene.publish(ps)
            self.get_logger().info(f"📡 已发布 {self.column_count} 个立柱障碍物")
            r.set("obstacles", json.dumps(self.obstacles))
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
        #waypoint = (ox + perp[0]*offset, oy + perp[1]*offset)
        if oy>0:
            waypoint = (ox -offset, oy-offset)
        else:
            waypoint = (ox -0.1, oy+offset)

        self.get_logger().info(
            f"🧭 绕行方向 {'左侧' if side > 0 else '右侧'} → waypoint=({waypoint[0]:.3f},{waypoint[1]:.3f})"
        )
        return waypoint

    def findpath(self, current, target,isPickup=True):
        """
        在 (current, target, origin) 三角区域内寻找障碍，
        并按角度排序生成自然路径。
        """
        origin = (0.0, 0.0)
        valid_obs = []
        waypoints = []
        redis_obstacles =json.loads(r.get('obstacles'))
        self.get_logger().info(f'📡 从 Redis 获取  个障碍物:{redis_obstacles},target:={target}')
        
        # --- 1️⃣ 筛选三角形内的障碍 ---
        for obs in redis_obstacles:
            if self.point_in_arc(obs, current, target):
                valid_obs.append(obs)

        if not valid_obs:
            self.get_logger().info("✅ 三角区域内无障碍")


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
        

        # 3️⃣ 直接计算绕行点（不再聚类）
        for obs, theta, _ in obs_angles:
            waypoint = self.compute_waypoint(obs, current, target)
            waypoints.append(waypoint)

        # 4️⃣ 平滑路径
        #for i in range(1, len(waypoints) - 1):
        #    wx = (waypoints[i-1][0] + waypoints[i][0] + waypoints[i+1][0]) / 3
        #    wy = (waypoints[i-1][1] + waypoints[i][1] + waypoints[i+1][1]) / 3
        #    waypoints[i] = (wx, wy)

        #  追加目标点
        if isPickup:
            waypoints.append(target)
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

    def plan_and_print_once(self,x,y,z,isPickup=True):
        #if self.planned_once:
        #    return
        #self.planned_once = True
        theta_init = [0, 0, 0]
        self.robot.get_scara_param()
        self.robot.wait_stop()
        self.get_logger().info('self.robot.x={:.3f},self.robot.y={:.3f},z={:.3f}'.format(self.robot.x,self.robot.y,self.robot.z)) 
        current_angle=[self.robot.angle1,self.robot.angle2,self.robot.r]
        xy=self.solver.check_forward_kinematics_from_gripper_position(np.deg2rad(current_angle))
        self.get_logger().info('gripper.x={:.3f},gripper.y={:.3f},target:{},{}'.format(xy[0],xy[1],x,y)) 
        waypoints=[]#self.findpath((xy[0],xy[1]),(x,y),isPickup)
        waypoints.append((x,y))
        for wp in waypoints:
            self.get_logger().info(f"➡️ 规划路径点: x={wp[0]:.3f}, y={wp[1]:.3f}")
            target = [wp[0], wp[1]]


            distance = math.sqrt(target[0] ** 2 + target[1] ** 2)
            if distance<=0.86:
                response=self.solver.get_robot_angle_in_degree(target)
                if response is not None:
                    joint_positions = response
                    forward_xy=self.solver.check_forward_kinematics_from_angles(np.deg2rad(joint_positions))
                    #self.get_logger().info('✅ IK Computed Successfully!joint_pos={}'.format(joint_positions))
                    self.get_logger().info(' Target: x={:.3f},y={:.3f},z={:.3f}, Forward FK: x={:.3f},y={:.3f}'.format(x,y,z,forward_xy[0],forward_xy[1]))
                    #self.get_logger().info(' Move to: angle1={:.1f},angle2={:.1f},angle3={:.1f},z={:.1f}'.format(joint_positions[0],joint_positions[1],joint_positions[2],z))
                    ret=self.robot.movej_angle(joint_positions[0],joint_positions[1],0,joint_positions[2]-0,30,1) 
                else:
                    self.get_logger().error(f'❌ IK computation failed: code={response.error_code.val}')
        self.robot.wait_stop()

        return waypoints
    def offset_plan_and_print_once(self,x,y,z,isPickup=True):
        #if self.planned_once:
        #    return
        #self.planned_once = True
        theta_init = [0, 0, 0]
        self.robot.get_scara_param()
        self.robot.wait_stop()
        self.get_logger().info('self.robot.x={:.3f},self.robot.y={:.3f},z={:.3f}'.format(self.robot.x,self.robot.y,self.robot.z)) 
        current_angle=[self.robot.angle1,self.robot.angle2,self.robot.r]
        xy=self.solver.check_forward_kinematics_from_gripper_position(np.deg2rad(current_angle))
        self.get_logger().info('gripper.x={:.3f},gripper.y={:.3f},target:{},{}'.format(xy[0],xy[1],x,y)) 
        waypoints=[]#self.findpath((xy[0],xy[1]),(x,y),isPickup)
        waypoints.append((xy[0]+x,xy[1]+y))
        for wp in waypoints:
            self.get_logger().info(f"➡️ 规划路径点: x={wp[0]:.3f}, y={wp[1]:.3f}")
            target = [wp[0], wp[1]]
            distance = math.sqrt(target[0] ** 2 + target[1] ** 2)
            if distance<=0.86:
                response=self.solver.get_robot_angle_in_degree(target)
                if response is not None:
                    joint_positions = response
                    forward_xy=self.solver.check_forward_kinematics_from_angles(np.deg2rad(joint_positions))
                    #self.get_logger().info('✅ IK Computed Successfully!joint_pos={}'.format(joint_positions))
                    self.get_logger().info(' Target: x={:.3f},y={:.3f},z={:.3f}, Forward FK: x={:.3f},y={:.3f}'.format(x,y,z,forward_xy[0],forward_xy[1]))
                    #self.get_logger().info(' Move to: angle1={:.1f},angle2={:.1f},angle3={:.1f},z={:.1f}'.format(joint_positions[0],joint_positions[1],joint_positions[2],z))
                    ret=self.robot.movej_angle(joint_positions[0],joint_positions[1],0,joint_positions[2]-0,30,1) 
                else:
                    self.get_logger().error(f'❌ IK computation failed: code={response.error_code.val}')
        self.robot.wait_stop()
        
        self.robot.get_scara_param()
        self.robot.wait_stop()
        self.get_logger().info('self.robot.x={:.3f},self.robot.y={:.3f},z={:.3f}'.format(self.robot.x,self.robot.y,self.robot.z)) 
        current_angle=[self.robot.angle1,self.robot.angle2,self.robot.r]
        new_xy=self.solver.check_forward_kinematics_from_gripper_position(np.deg2rad(current_angle))
        ret=math.sqrt((new_xy[0]-wp[0])**2+(new_xy[1]-wp[1])**2)
        self.get_logger().info('After adj Move: Target: x={:.3f},y={:.3f}, Forward FK: x={:.3f},y={:.3f}, distance={:.3f}'.format(wp[0],wp[1],new_xy[0],new_xy[1],ret))

        return ret


     

    def publish_path(self, path):
        msg = Path()
        msg.header.frame_id = 'base_link'   # ✅ 一定要写对（map 或 odom）
        msg.header.stamp = self.get_clock().now().to_msg()
        #self.get_logger().info(f"path:{path} ")
        for x, y in path:
            pose = PoseStamped()
            pose.header.frame_id = 'base_link'    # ✅ 和上面一致
            pose.pose.position.x = x*1.0
            pose.pose.position.y = y*1.0
            pose.pose.position.z = 0.0
            msg.poses.append(pose)
        self.pub_path.publish(msg)
        self.get_logger().info(f"📡 已发布路径到 /rrt_path，共 {len(path)} 点")

    def move(self,goal):
        redis_obstacles = json.loads(r.get('obstacles'))
        self.obstacles=redis_obstacles if redis_obstacles else self.obstacles
        self.get_logger().info(f"move to goal:{goal},self.obstacles:{self.obstacles}")
        self.robot.get_scara_param()
        self.robot.wait_stop()
        gripper_rotation = ((self.robot.r-self.robot.angle1-self.robot.angle2 + 180) % 360) - 180
        current_angle=[self.robot.angle1*3.14159/180,self.robot.angle2*3.14159/180,gripper_rotation*3.14159/180]
        self.rrt_planner.set_start_pos(current_angle)
        self.rrt_planner.obstacles.clear()

        for obs in self.obstacles:
            self.rrt_planner.add_obstacle('circle', (obs[0], obs[1], 0.1))
        path,joint_path=self.rrt_planner.build_tree(goal) 
        #if path:
        #    self.publish_path(path)
        robot_angles=[self.rrt_planner.get_robot_angle_in_degree(p) for p in joint_path] if joint_path else None
        for robot_angle in robot_angles if robot_angles else []:
            ret=self.robot.new_movej_angle(robot_angle[0],robot_angle[1],0,robot_angle[2]-0,30,1) 
        self.robot.wait_stop()
        self.get_logger().info(f"RRT waypoints: {len(joint_path) if joint_path else 0}")  
        return     robot_angles


    def bounding_boxes_callback(self, msg):
        #r.set("gripper_flag", "ready")
        if r.get("mode")=="camera_ready":
            mushroom_xyz=msg.data
            mushroom_xyz=msg.data.split(",");
            #mushroom_xyz=r.get("mushroom_xyz")
            if mushroom_xyz is None :
                self.get_logger().info(f"no mushroom_xyz")
                return
            #goal=[int(float(mushroom_xyz[2].strip())),0-int(float(mushroom_xyz[0].strip()))] # this is from yolox_ros
            goal=[float(mushroom_xyz[2].strip()),0-float(mushroom_xyz[0].strip())]
            self.get_logger().info(f"get mushroom_xyz {mushroom_xyz},goal:{goal}")    
            if abs(goal[0]) >0.8 or abs(goal[1])>0.8 or (abs(goal[0])<0.35 and abs(goal[1])<0.35):
                self.get_logger().info(f"mushroom_xyz out of range:{mushroom_xyz},goal:{goal}")    
                return
            #self.move_robo_angles=self.move(goal)
            self.plan_and_print_once(x=goal[0],y=goal[1],z=0.1,isPickup=True)

            time.sleep(2)

        r.set("mode","adjust_ready")    
        r.set("gripper_flag", "done")
        r.set("mushroom_xyz","")    

    def adj_bounding_boxes_callback(self, msg):
        if r.get("mode") == "adjust_ready":
            mushroom_xyz = msg.data
            mushroom_xyz = msg.data.split(",")
            goal = [float(mushroom_xyz[0].strip()), float(mushroom_xyz[1].strip()), float(mushroom_xyz[2].strip())]
            self.robot.get_scara_param()
            dist_incamera=math.sqrt(goal[0]*goal[0]+goal[1]*goal[1])
            if (dist_incamera>0.03):#(abs(self.robot.x - goal[2])>50 or abs(self.robot.y - goal[0])>50):
                #this is for adj from top camera
                #RX=-goal[1]
                #RY=-goal[0]
                #RZ=goal[2]
                RX=goal[0]
                RY=-goal[1]
                RZ=goal[2]
                X=RX*math.cos(math.radians(self.hitbot_r))-RY*math.sin(math.radians(self.hitbot_r))
                Y=RX*math.sin(math.radians(self.hitbot_r))+RY*math.cos(math.radians(self.hitbot_r))
                ret_distance=self.offset_plan_and_print_once(x=X,y=Y,z=0.1,isPickup=True)
                self.get_logger().info(f"adj bounding_boxes_callback :{goal},x_offset:{X},y_offset:{Y},dist_incamera:{dist_incamera}")
                r.set("mode", "adjust_ready")
                return

                # this is for adj from side camera
                #ret = self.robot.movej_xyz(self.robot.x - np.sign(goal[2])*20, self.robot.y - np.sign(goal[0])*20, self.robot.z, self.robot.r, 50, 1)
                #self.robot.wait_stop()
                #self.get_logger().info(f"adj bounding_boxes_callback ->adjust :{goal},x_offset:{goal[2]},y_offset:{goal[0]},ret :{ret}")
            if 1:  # ret<2:
                r.set("mode", "adjust_done")
                #response = self.client_node.open_send_request()
                # if response is not None:
                self.robot.get_scara_param()
                self.robot.wait_stop()
                ret = self.robot.movej_xyz(self.robot.x, self.robot.y, self.robot.z - 150, self.robot.r, 30, 1)
                self.robot.wait_stop()
                self.get_logger().info(f"move in -z, ret :{ret}")
                #response = self.client_node.send_request()  # close request
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
                self.robot.wait_stop()
                ret = self.robot.movej_xyz(self.robot.x, self.robot.y, 0, self.robot.r, 50, 1)
                self.robot.wait_stop()
                time.sleep(1)

                #self.plan_and_print_once(x=0,y=0.8,z=0,isPickup=True)
                self.robot.get_scara_param()
                ret=self.robot.movej_xyz(600,0,0,0,50,1)
                self.robot.wait_stop()
                self.move_robo_angles=self.move([0.10,0.6])
                self.robot.get_scara_param()
                ret=self.robot.movej_xyz(0,400,0,90+90,50,1)
                self.robot.wait_stop()
                #response = self.client_node.open_send_request()
                self.move_robo_angles=self.move(np.array([0.5,0.1]))
                self.move_robo_angles=[]
            # else:
            #     r.set("mode","camera_ready")
            r.set("mode", "camera_ready")

        time.sleep(2)
        r.set("gripper_flag", "done")

  


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
    def thread_check_collision_callback(self):
        threading.Thread(target=self.check_collision_callback, daemon=True).start()
    def check_collision_callback(self):
        nearest, dist = self.find_nearest_obstacle()
        if nearest:
            self.get_logger().info(f"最近障碍点: {nearest}, 距离: {dist:.3f} m")
            if dist < 0.15:
                self.get_logger().warn("⚠️ 可能碰撞！")
                self.robot.stop_move()

    def publish_joint_states(self):
        # Get real joint positions from HitBot API (Replace this with actual API calls)
        self.robot.get_scara_param()
        self.robot.wait_stop()
        joint_positions = [self.robot.z,self.robot.angle1*3.14/180,self.robot.angle2*3.14/180,(self.robot.r)*3.14/180]

        # Create JointState message
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = self.joint_names
        joint_state_msg.position = joint_positions
        # Publish the joint states
        #self.get_logger().info(f"publish_joint_states :joint_positions={joint_positions}")
        self.joint_state_pub.publish(joint_state_msg)

        self.hitbot_x=self.robot.x/1000
        self.hitbot_y=self.robot.y/1000
        self.hitbot_z=self.robot.z/1000

        self.publish_hitbot_x(str(int(self.robot.x)))
        self.publish_hitbot_y(str(int(self.robot.y)))
        self.publish_hitbot_z(str(int(self.robot.z)))
        self.publish_hitbot_r(str(int(self.robot.r)))
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
                angle = (waypoints[3] - waypoints[2] - waypoints[1]) * 180 / math.pi
                # 归一化到 [-180, 180)
                angle = (angle + 180) % 360 - 180
                self.robot.new_movej_angle(waypoints[1]*180/3.14, waypoints[2]*180/3.14, waypoints[0], (angle)-0.0, 50, 1)
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
