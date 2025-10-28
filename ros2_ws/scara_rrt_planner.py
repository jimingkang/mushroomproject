import numpy as np
import math, random
import matplotlib.pyplot as plt

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

from scipy.optimize import root

L1, L2, L3 = 0.325, 0.275, 0.26


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
        #if abs(self.normalize_angle(ans[2])*180/math.pi)>150:
        #    print("ERROR: potential collision for gripper to link (IK failed)!")
        #    return [0.0, 0.0, 0.0]
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


class ScaraRRTPlanner(Node):
    def __init__(self):
        super().__init__("ScaraRRTPlanner")
        self.sub_scan = self.create_subscription(LaserScan, "/scan", self.scan_callback, 10)
        self.pub_scene = self.create_publisher(PlanningScene, "/planning_scene", 10)
        self.pub_path = self.create_publisher(Path, '/rrt_path', 10)
        # ---------- 参数 ----------
        self.L1, self.L2, self.L3 = 0.325, 0.275, 0.254
        self.safe_dist = 0.05
        self.step_size = 0.2   # 弧度步长
        self.max_iter = 2000
        self.obstacles = [(0.45, 0.10), (0.40, -0.15), (0.55, 0.25)]

        self.fk_cache={}
    
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
            if 0.4 < dist < 1.5:
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

        self.obstacles.append((0.4, 0.4))
      # 输出结果
        self.get_logger().info(f"检测到 {len(self.obstacles)} 个障碍物")
        for i, (cx, cy) in enumerate(self.obstacles):
            if cy>0:
                self.addobject(ps, cx, cy)
                self.get_logger().info(f"障碍物 {i+1}: x={cx:.2f}, y={cy:.2f}")
        # ✅ 发布所有立柱到规划场景
        if self.column_count > 0:
            self.pub_scene.publish(ps)
            self.get_logger().info(f"📡 已发布 {self.column_count} 个立柱障碍物")
        #res=self.findpath((1.8,0),(0.0,1.8))
        #print("red:{}",res)
        start=self.solver.get_robot_angle_in_degree((0.6,0))
        start_rad=tuple(math.radians(d) for d in start)
        end=self.solver.get_robot_angle_in_degree((0.0,0.6))
        end_rad=tuple(math.radians(d) for d in end)
        path = self.run_rrt_joint(start_rad, end_rad)
        if path:
            self.publish_path(path)
            print("rrt:{}",path)

    # ---------- 几何 ----------
    def fk(self, q):
        θ1, θ2, θ3 = q
        x1 = self.L1 * math.cos(θ1)
        y1 = self.L1 * math.sin(θ1)
        x2 = x1 + self.L2 * math.cos(θ1 + θ2)
        y2 = y1 + self.L2 * math.sin(θ1 + θ2)
        x3 = x2 + self.L3 * math.cos(θ1 + θ2 + θ3)
        y3 = y2 + self.L3 * math.sin(θ1 + θ2 + θ3)
        return np.array([[0, x1, x2, x3], [0, y1, y2, y3]])

    def point_to_segment_distance(self, p, a, b):
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

    def scara_in_collision(self, q):
        pts = self.fk(q)
        for (ox, oy) in self.obstacles:
            for i in range(3):
                a = (pts[0,i], pts[1,i])
                b = (pts[0,i+1], pts[1,i+1])
                if self.point_to_segment_distance((ox,oy), a, b) < self.safe_dist:
                    return True
        return False

    def workspace_dist(self, q1, q2):
        """
        比较两个关节姿态对应的末端 (x, y) 距离
        """
        xs1, ys1 = self.forward_kinematics(q1)
        xs2, ys2 = self.forward_kinematics(q2)
        ee1 = np.array([xs1[-1], ys1[-1]])
        ee2 = np.array([xs2[-1], ys2[-1]])
        return np.linalg.norm(ee1 - ee2)

    # ---------- RRT 工具 ----------
    def dist(self, a,b):
        return np.linalg.norm(np.array(a)-np.array(b))

    def interpolate_joints(self, q1, q2, steps=10):
        q1, q2 = np.array(q1), np.array(q2)
        return [tuple(q1 + (q2-q1)*t) for t in np.linspace(0,1,steps)]

    def run_rrt_joint(self, q_start, q_goal):
        nodes = [q_start]
        parents = {q_start: None}
        for _ in range(self.max_iter):
            q_rand = (random.uniform(-math.pi, math.pi),
                      random.uniform(-math.pi, math.pi),
                      random.uniform(-math.pi, math.pi))
            #q_near = min(nodes, key=lambda q: self.dist(q, q_rand))
            q_near = min(  nodes,key=lambda q: self.workspace_dist(q, q_rand))
            direction = np.array(q_rand) - np.array(q_near)
            norm = np.linalg.norm(direction)
            if norm == 0:
                continue
            q_new = tuple(np.array(q_near) + direction/norm * self.step_size)

            # 检查插补碰撞
            safe = True
            for q in self.interpolate_joints(q_near, q_new, steps=8):
                if self.scara_in_collision(q):
                    safe = False
                    break
            if not safe:
                continue

            nodes.append(q_new)
            parents[q_new] = q_near

            #if self.dist(q_new, q_goal) < 0.2:
            if self.workspace_dist(q_new, q_goal) < 0.05:
                print("✅ 找到路径!")
                return self.reconstruct(parents, q_new)
        return None
    def forward_kinematics(self, q):
        key = tuple(np.round(q, 3))
        if key in self.fk_cache:
            return self.fk_cache[key]

        x0, y0 = 0, 0
        x1 = x0 + self.L1 * math.cos(q[0])
        y1 = y0 + self.L1 * math.sin(q[0])
        x2 = x1 + self.L2 * math.cos(q[0] + q[1])
        y2 = y1 + self.L2 * math.sin(q[0] + q[1])
        x3 = x2 + self.L3 * math.cos(q[0] + q[1] + q[2])
        y3 = y2 + self.L3 * math.sin(q[0] + q[1] + q[2])

        result = np.array([[x0, x1, x2, x3], [y0, y1, y2, y3]])
        self.fk_cache[key] = result
        return result

    def reconstruct(self, parents, node):
        path = [node]
        while node is not None:
            node = parents[node]
            if node:
                path.append(node)
        path.reverse()
        return path

    # ---------- 平滑 ----------
    def shortcut_smooth(self, joint_path):
        if len(joint_path) < 3:
            return joint_path
        smooth = [joint_path[0]]
        last = joint_path[0]
        for i in range(2, len(joint_path)):
            safe = True
            for q in self.interpolate_joints(last, joint_path[i], steps=10):
                if self.scara_in_collision(q):
                    safe = False
                    break
            if not safe:
                smooth.append(joint_path[i-1])
                last = joint_path[i-1]
        smooth.append(joint_path[-1])
        return smooth

    # ---------- 动画显示 ----------
    def visualize(self, joint_path):
        fig, ax = plt.subplots()
        ax.set_aspect('equal')
        ax.set_xlim(-0.2, 0.8)
        ax.set_ylim(-0.3, 0.6)
        ax.set_title("Joint-space RRT with Collision Check")
        for (ox, oy) in self.obstacles:
            ax.add_patch(plt.Circle((ox,oy), 0.03, color='blue'))
        link_lines = [plt.plot([], [], 'g-', lw=4)[0] for _ in range(3)]
        trace_x, trace_y = [], []
        trace_line, = plt.plot([], [], 'r--', lw=1)

        for q in joint_path:
            pts = self.fk(q)
            for j in range(3):
                link_lines[j].set_data([pts[0,j], pts[0,j+1]], [pts[1,j], pts[1,j+1]])
            trace_x.append(pts[0,-1])
            trace_y.append(pts[1,-1])
            trace_line.set_data(trace_x, trace_y)
            plt.pause(0.03)
        plt.show()

    # ---------- 主入口 ----------
    def run(self):
        q_start = (0.0, 0.0, 0.0)
        q_goal = (0.8, -0.4, 0.2)
        joint_path = self.run_rrt_joint(q_start, q_goal)
        if not joint_path:
            print("❌ 无路径")
            return
        joint_path = self.shortcut_smooth(joint_path)
        print(f"关节路径长度: {len(joint_path)}")
        self.visualize(joint_path)


def main():
    rclpy.init()
    node = ScaraRRTPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
