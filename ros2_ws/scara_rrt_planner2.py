

import numpy as np
from scipy.spatial import KDTree
from math import cos, sin, pi, sqrt
import random
import matplotlib.pyplot as plt
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




class ScaraRRTPlanner2(Node):
    def __init__(self):
        super().__init__("ScaraRRTPlanner2")
        self.sub_scan = self.create_subscription(LaserScan, "/scan", self.scan_callback, 10)
        self.pub_scene = self.create_publisher(PlanningScene, "/planning_scene", 10)
        self.pub_path = self.create_publisher(Path, '/rrt_path', 10)
        # ---------- 参数 ----------
        # --- 机械臂参数 ---
        self.L1, self.L2, self.L3 = 0.345, 0.25, 0.254
        self.workspace_size = self.L1 + self.L2 + self.L3

        # --- RRT参数 ---
        self.step_size = 0.08
        self.max_iter = 2000
        self.tolerance = 0.03

        # --- KDTree 数据 ---
        self.joint_nodes = []
        self.ee_points = []
        self.kdtree = None

        # --- 结构记录 ---
        self.parents = {}
        self.fk_cache = {}

       


        self.obstacles = [(0.45, 0.10),  (0.55, 0.25)]
        #self.solver=OldSolver()
    
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
        q_start = (0, 0, 0)
        goal_ws = np.array([0.0, 0.65])
        path = self.run_rrt_joint_with_kdtree(q_start, goal_ws)
        if path:
            self.publish_path(path)
            print("rrt:{}",path)


    # -----------------------------------------------------
    # 正向运动学：计算各关节位置
    # -----------------------------------------------------
    def forward_kinematics(self, q):
        key = tuple(np.round(q, 3))
        if key in self.fk_cache:
            return self.fk_cache[key]

        x0, y0 = 0, 0
        x1 = x0 + self.L1 * cos(q[0])
        y1 = y0 + self.L1 * sin(q[0])
        x2 = x1 + self.L2 * cos(q[0] + q[1])
        y2 = y1 + self.L2 * sin(q[0] + q[1])
        x3 = x2 + self.L3 * cos(q[0] + q[1] + q[2])
        y3 = y2 + self.L3 * sin(q[0] + q[1] + q[2])

        result = np.array([[x0, x1, x2, x3], [y0, y1, y2, y3]])
        self.fk_cache[key] = result
        return result

    # -----------------------------------------------------
    # 连杆碰撞检测（线段 vs 圆/矩形）
    # -----------------------------------------------------
    def scara_in_collision(self, q):
        xs, ys = self.forward_kinematics(q)
        for x1, y1, x2, y2 in zip(xs, ys, xs[1:], ys[1:]):
            for shape, params in self.obstacles:
                if shape == "circle":
                    cx, cy, r = params
                    if self.line_circle_collision(x1, y1, x2, y2, cx, cy, r):
                        return True
                elif shape == "rect":
                    rx, ry, rw, rh = params
                    if self.line_rect_collision(x1, y1, x2, y2, rx, ry, rw, rh):
                        return True
        return False

    def line_circle_collision(self, x1, y1, x2, y2, cx, cy, r):
        d = np.array([x2 - x1, y2 - y1])
        f = np.array([x1 - cx, y1 - cy])
        a = np.dot(d, d)
        b = 2 * np.dot(f, d)
        c = np.dot(f, f) - r * r
        discriminant = b * b - 4 * a * c
        if discriminant < 0:
            return False
        disc = sqrt(discriminant)
        t1 = (-b - disc) / (2 * a)
        t2 = (-b + disc) / (2 * a)
        return (0 <= t1 <= 1) or (0 <= t2 <= 1)

    def line_rect_collision(self, x1, y1, x2, y2, rx, ry, rw, rh):
        for t in np.linspace(0, 1, 20):
            xt = x1 + t * (x2 - x1)
            yt = y1 + t * (y2 - y1)
            if rx <= xt <= rx + rw and ry <= yt <= ry + rh:
                return True
        return False

    # -----------------------------------------------------
    # 连续插补检测
    # -----------------------------------------------------
    def interpolate_joints(self, q1, q2, steps=10):
        q1, q2 = np.array(q1), np.array(q2)
        return [tuple(q1 + (q2 - q1) * t) for t in np.linspace(0, 1, steps)]

    # -----------------------------------------------------
    # goal-biased 采样
    # -----------------------------------------------------
    def sample_position(self, goal_ws=None, goal_bias=0.2):
        if goal_ws is not None and np.random.rand() < goal_bias:
            for _ in range(10):
                sample = [np.random.uniform(-pi/2, pi/2),
                          np.random.uniform(-3*pi/4, 3*pi/4),
                          np.random.uniform(-3*pi/4, 3*pi/4)]
                xs, ys = self.forward_kinematics(sample)
                ee = np.array([xs[-1], ys[-1]])
                if np.linalg.norm(ee - goal_ws) < 0.2 * self.workspace_size:
                    return tuple(sample)
        return tuple(np.random.uniform([-pi/2, -3*pi/4, -3*pi/4],
                                       [pi/2,  3*pi/4,  3*pi/4]))

    # -----------------------------------------------------
    # KDTree 最近邻（在末端空间）
    # -----------------------------------------------------
    def nearest_node(self, goal_ws):
        if self.kdtree is None:
            return self.joint_nodes[0]
        dist, idx = self.kdtree.query(goal_ws)
        return self.joint_nodes[idx]

    # -----------------------------------------------------
    # RRT 核心算法（Joint空间采样 + Workspace KDTree）
    # -----------------------------------------------------
    def run_rrt_joint_with_kdtree(self, q_start, goal_ws):
        self.joint_nodes = [q_start]
        xs, ys = self.forward_kinematics(q_start)
        ee = np.array([xs[-1], ys[-1]])
        self.ee_points = [ee]
        self.kdtree = KDTree(self.ee_points)
        self.parents = {q_start: None}
        success = False

        for i in range(self.max_iter):
            q_rand = self.sample_position(goal_ws)
            ee_rand = np.array(self.forward_kinematics(q_rand))[:, -1]
            q_near = self.nearest_node(ee_rand)

            direction = np.array(q_rand) - np.array(q_near)
            norm = np.linalg.norm(direction)
            if norm == 0:
                continue
            q_new = tuple(np.array(q_near) + direction / norm * self.step_size)

            # 连续插补检测
            safe = True
            for q in self.interpolate_joints(q_near, q_new, steps=10):
                if self.scara_in_collision(q):
                    safe = False
                    break
            if not safe:
                continue

            # 添加节点
            self.joint_nodes.append(q_new)
            xs, ys = self.forward_kinematics(q_new)
            ee_new = np.array([xs[-1], ys[-1]])
            self.ee_points.append(ee_new)
            self.parents[q_new] = q_near
            if len(self.ee_points) % 20 == 0:
                self.kdtree = KDTree(self.ee_points)

            if np.linalg.norm(ee_new - goal_ws) < self.tolerance:
                print(f"✅ Goal reached in {i} iterations!")
                success = True
                break

        if not success:
            print("❌ Goal not reached.")
            return None

        # 回溯路径
        path = []
        node = q_new
        while node is not None:
            path.append(node)
            node = self.parents[node]
        path.reverse()

        # 平滑
        path = self.shortcut_smooth(path)
        return path

    # -----------------------------------------------------
    # 路径平滑（捷径平滑）
    # -----------------------------------------------------
    def shortcut_smooth(self, path):
        if len(path) < 3:
            return path
        smoothed = [path[0]]
        i = 0
        while i < len(path) - 1:
            j = len(path) - 1
            while j > i + 1:
                valid = True
                for q in self.interpolate_joints(path[i], path[j], steps=10):
                    if self.scara_in_collision(q):
                        valid = False
                        break
                if valid:
                    smoothed.append(path[j])
                    i = j
                    break
                j -= 1
            else:
                smoothed.append(path[i + 1])
                i += 1
        return smoothed

    # -----------------------------------------------------
    # 可视化（matplotlib）
    # -----------------------------------------------------
    def visualize(self, path):
        plt.figure(figsize=(5, 5))
        for shape, params in self.obstacles:
            if shape == "circle":
                cx, cy, r = params
                plt.gca().add_patch(plt.Circle((cx, cy), r, color="gray", alpha=0.4))
            elif shape == "rect":
                rx, ry, rw, rh = params
                plt.gca().add_patch(plt.Rectangle((rx, ry), rw, rh, color="gray", alpha=0.4))

        for q in path:
            xs, ys = self.forward_kinematics(q)
            plt.plot(xs, ys, "b-")
            plt.plot(xs[-1], ys[-1], "ro")

        plt.axis("equal")
        plt.title("SCARA RRT Path")
        plt.show()

    # -----------------------------------------------------
    # ROS2 Path 发布
    # -----------------------------------------------------
    def publish_path(self, path):
        if not self.pub_path:
            return
        msg = Path()
        msg.header.frame_id = 'base_link'
        msg.header.stamp = self.get_clock().now().to_msg()
        for q in path:
            xs, ys = self.forward_kinematics(q)
            pose = PoseStamped()
            pose.header.frame_id = 'base_link'
            pose.pose.position.x = float(xs[-1])
            pose.pose.position.y = float(ys[-1])
            msg.poses.append(pose)
            print(f"📡 发布{xs[-1]}{ys[-1]} 点:")
        self.pub_path.publish(msg)
        print(f"📡 发布路径: {len(path)} 点:")
        self.visualize(path)


# ---------------------------------------------------------
# ✅ 测试主函数
# ---------------------------------------------------------
def main():
    rclpy.init()
    node = ScaraRRTPlanner2()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
