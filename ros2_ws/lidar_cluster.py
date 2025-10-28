#!/usr/bin/env python3
import json
import random
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from moveit_msgs.msg import PlanningScene, CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
import math
import time
import uuid
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import redis
from scipy.optimize import root
from scipy.spatial import KDTree
from tqdm import tqdm
from scipy.interpolate import splprep, splev
import numpy as np

from matplotlib import pyplot as plt
redis_server='172.23.248.33'

pool = redis.ConnectionPool(host=redis_server, port=6379, decode_responses=True,password='jimmy')
red = redis.Redis(connection_pool=pool)

L1, L2, L3 = 0.325, 0.275, 0.26

class RRTAngle:
    def __init__(self, start_pos, obstacles=[], tolerance=0.1, step_size=0.2):
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
        x3 = x2 + self.length[1] * np.cos(angles[0] + angles[1]+angles[2])
        y3 = y2 + self.length[1] * np.sin(angles[0] + angles[1]+ angles[2])

        result = np.array([[x0, x1, x2,x3], [y0, y1, y2,y3]], dtype=np.float32)
        self.forward_kinematics_cache[angles_tuple] = result  # Cache the result
        return result

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
            min_idx = np.argmin(angle_cost)
            best_angle = ik_success[min_idx]
            return best_angle,angle_cost[min_idx]
        else:
            return None, np.inf
    

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
        best_angle, cost = self.solve(target, current_angle)
        return best_angle
        #solved, angle_cost = self.solve(target, current_angle)
        #min_config = np.argmin(angle_cost)
        #try:
        #    return solved[min_config]
        #except IndexError:
        #    print(f"Error: No solution at index {min_config} (total solutions: {len(solved)})")
        #    return None  # or raise a more descriptive exception

        #return solved[min_config]
    def normalize_angle(self,deg):
        deg = ((deg + 180) % 360) - 180
        return round(deg, 1)
    def normalize_angle_rad(self,rad):
        rad = (rad + math.pi) % (2 * math.pi) - math.pi
        return round(rad, 6)   # rounding optional

    def get_robot_angle_in_degree(self, target, current_angle=[0,0,0]):
        ans=self.get_angle_to_target(target, current_angle)

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
    def get_robot_angle_in_rad(self, target, current_angle=[0,0,0]):
        ans=self.get_angle_to_target(target, current_angle)
        if ans is None:  # Check if IK failed
            print("ERROR: No valid joint angles (IK failed)!")
            return None  # Default safe value (or raise an exception)
        print(f" IK angles :{ans}")
        theta1 = ans[0] 
        theta2 = ans[1] 
        theta3 = ans[2] 


    # 累积角度
        angles = [theta1, theta2,theta1 + theta2 + theta3]
        normalized = [self.normalize_angle_rad(a) for a in angles]
        return normalized

class ScanToCylinders(Node):
    def __init__(self):
        super().__init__("scan_to_cylinders")
        self.sub_scan = self.create_subscription(LaserScan, "/scan", self.scan_callback, 10)
        self.pub_scene = self.create_publisher(PlanningScene, "/planning_scene", 10)
        self.pub_path = self.create_publisher(Path, '/rrt_path', 10)
        self.get_logger().info("✅ 已启动: 将 /scan 转换为 3D 细小立柱障碍物")
        self.obstacles = []  # 存储障碍物点
        self.column_count = 0

                # 参数
        self.step_size = 0.1
        self.max_iter = 500
        self.goal_tolerance = 0.1
        self.solver = OldSolver()

        self.rrt_planner = RRTAngle(start_pos=[0.0, 0.0, 0.0], obstacles=[], tolerance=self.goal_tolerance, step_size=self.step_size)
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
        x_min = min(current[0], target[0]) - margin
        x_max = max(current[0], target[0]) + margin
        y_min = min(current[1], target[1]) - margin
        y_max = max(current[1], target[1]) + margin

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
# ✅ 创建圆柱体障碍物
    def addobject(self,ps,x,y):
        co = CollisionObject()
        co.id = "pillar_" + str(uuid.uuid4())[:8]
        co.header.frame_id = "base_link"
        co.operation = CollisionObject.ADD
        cylinder = SolidPrimitive()
        cylinder.type = SolidPrimitive.CYLINDER
        cylinder.dimensions = [0.5, 0.05]  # 高0.5m 半径2cm

        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = 0.25  # 中心点高度
        pose.orientation.w = 1.0

        co.primitives.append(cylinder)
        co.primitive_poses.append(pose)
        ps.world.collision_objects.append(co)
        self.column_count += 1


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
            if self.point_in_square(obs, current, target):
                valid_obs.append(obs)

        if not valid_obs:
            self.get_logger().info("✅ 区域内无障碍")
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
        

        # 3️⃣ 直接计算绕行点（不再聚类）
        for obs, theta, _ in obs_angles:
            waypoint = self.compute_waypoint(obs, current, target)
            waypoints.append(waypoint)

        # 4️⃣ 平滑路径
        for i in range(1, len(waypoints) - 1):
            wx = (waypoints[i-1][0] + waypoints[i][0] + waypoints[i+1][0]) / 3
            wy = (waypoints[i-1][1] + waypoints[i][1] + waypoints[i+1][1]) / 3
            waypoints[i] = (wx, wy)

        #  追加目标点
        waypoints.append(target)
        self.get_logger().info(f"✅ 找到 {len(waypoints)} 个绕行点（按角度排序）")
        return waypoints
    def scan_callback(self, msg: LaserScan):
        self.get_logger().info(f"角度范围: {msg.angle_min:.2f} ~ {msg.angle_max:.2f}, 角度增量: {msg.angle_increment:.4f}")
        ps = PlanningScene()
        ps.is_diff = True
        ps.world.collision_objects.clear()
        self.column_count = 0
        self.obstacles.clear()
        self.obstacles=[]
        points = []

        angle = msg.angle_min
        for i, r in enumerate(msg.ranges):
            if not (msg.range_min < r < msg.range_max):
                continue

            angle = msg.angle_min + i * msg.angle_increment
            x = r * math.cos(angle)
            y = r * math.sin(angle)
            P = (x, y)
            dist = math.sqrt(x**2 + y**2)
            if 0.4 < dist < 1:
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
      # 输出结果
        self.get_logger().info(f"检测到 {len(self.obstacles)} 个障碍物")
        for i, (cx, cy) in enumerate(self.obstacles):
            self.addobject(ps, cx, cy)
            self.get_logger().info(f"障碍物 {i+1}: x={cx:.2f}, y={cy:.2f}")
        
        #self.addobject(ps, 0.5, 0.3)
        # ✅ 发布所有立柱到规划场景
        if self.column_count > 0:
            self.pub_scene.publish(ps)
            self.get_logger().info(f"📡 已发布 {self.column_count} 个立柱障碍物,{str(self.obstacles)}")
            red.set("obstacles", json.dumps(self.obstacles))
        #res=self.findpath((1.8,0),(0.0,1.8))
        #print("red:{}",res)
        self.rrt_planner.set_start_pos([0.0,0.0,0.0])
        self.rrt_planner.obstacles.clear()
        for obs in self.obstacles:
            self.rrt_planner.add_obstacle('circle', (obs[0], obs[1], 0.2))
        #self.rrt_planner.add_obstacle('circle', (0.5, 0.3, 0.2))
        #path,smooth_joint_path=self.rrt_planner.build_tree([0.0,0.4])
        #robot_angles=[self.rrt_planner.get_robot_angle_in_degree(p) for p in smooth_joint_path] if smooth_joint_path else None
        #self.get_logger().info(f"RRT路径点数: {len(path) if path else 0}")
        #if path:
        #    self.publish_path(path)

        time.sleep(3)

    def run_rrt(self,start, goal):
        nodes = [start]
        parents = {start: None}
        for _ in range(self.max_iter):
            rand = (random.uniform(0, 0.9), random.uniform(-0.9, 0.9))
            nearest = min(nodes, key=lambda n: self.dist(n, rand))
            angle = math.atan2(rand[1]-nearest[1], rand[0]-nearest[0])
            new_node = (nearest[0]+self.step_size*math.cos(angle),
                        nearest[1]+self.step_size*math.sin(angle))
            if not self.path_collision_free(nearest, new_node): continue
            nodes.append(new_node)
            parents[new_node] = nearest
            if self.dist(new_node, goal) < 0.1:
                return self.reconstruct_path(parents, new_node,start)
        return None
    def run_rrt_kdtree(self, start, goal): 
        self.nodes = [start]
        self.kdtree = KDTree(self.nodes)
        parents = {start: None}

        for _ in range(self.max_iter):
            rand = (random.uniform(0, 0.8), random.uniform(-0.8, 0.8))

            # ✅ 用 KDTree 查最近点
            _, idx = self.kdtree.query(rand)
            nearest = self.nodes[idx]

            new_node = self.steer(nearest, rand)

            if not self.path_collision_free(nearest, new_node):
                continue

            self.nodes.append(new_node)
            parents[new_node] = nearest

            # ✅ 每次更新 KDTree
            self.kdtree = KDTree(self.nodes)

            if self.dist(new_node, goal) < 0.1:
                raw_path = self.reconstruct_path(parents, new_node, start)
                smooth_path = self.smooth_path(raw_path)
                return smooth_path

        return None
    def steer(self, from_node, to_node):
        dx, dy = to_node[0]-from_node[0], to_node[1]-from_node[1]
        d = math.hypot(dx, dy)
        if d < self.step_size:
            return to_node
        scale = self.step_size / d
        return (from_node[0] + dx*scale, from_node[1] + dy*scale)

    def in_collision(self, point):
        for ox, oy in self.obstacles:
            if self.dist(point, (ox, oy)) < 0.15:
                return True
        return False

    def reconstruct_path(self, parents, node, start):
        path = []
        while node is not None:
            path.append(node)
            node = parents[node]
        path.reverse()
        path.insert(0, start)
        return path
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

        # ---------- 工具函数 ----------
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

    def scara_in_collision(self,q):
        pts = self.fk(q)
        for (ox, oy) in self.obstacles:
            for i in range(3):
                a = (pts[0, i], pts[1, i])
                b = (pts[0, i+1], pts[1, i+1])
                d = self.point_to_segment_distance((ox, oy), a, b)
                if d < 0.15:
                    return True
        return False

    # ---------- 逆运动学 (2D 平面近似解) ----------
    def ik_solutions(self,x, y):
        """ 返回一个可能的逆解 (θ1, θ2, θ3=0) """
        # 逆运动学简化为 planar 2-link 解
        L12 = L1 + 1e-6
        L22 = L2 + L3 + 1e-6
        r = math.hypot(x, y)
        if r > L12 + L22:
            return None
        cos2 = (r**2 - L12**2 - L22**2) / (2 * L12 * L22)
        cos2 = max(-1, min(1, cos2))
        θ2 = math.acos(cos2)
        k1 = L12 + L22 * math.cos(θ2)
        k2 = L22 * math.sin(θ2)
        θ1 = math.atan2(y, x) - math.atan2(k2, k1)
        return (θ1, θ2, 0.0)

    # ---------- RRT ----------
    def dist(self,a, b):
        return math.hypot(a[0]-b[0], a[1]-b[1])

    def collision_free(self,p1, p2):
        # 检查路径线段是否离障碍太近
        for (ox, oy) in self.obstacles:
            if self.point_to_segment_distance((ox,oy), p1, p2) < 0.1:
                return False
        return True
    def path_collision_free(self, p1, p2, steps=3):
        """沿路径插补并检测机械臂连杆是否碰撞"""
        for i in range(steps + 1):
            self.get_logger().info(f"i:{i},p1={p1}, ")
            t = i / steps
            x = p1[0] + (p2[0] - p1[0]) * t
            y = p1[1] + (p2[1] - p1[1]) * t
            ik_sol = self.solver.get_robot_angle_in_rad([x, y],current_angle=[0,0,0])
            if ik_sol is None:
                return False
            if self.scara_in_collision(ik_sol):
                return False
        return True


def main():
    rclpy.init()
    node = ScanToCylinders()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

