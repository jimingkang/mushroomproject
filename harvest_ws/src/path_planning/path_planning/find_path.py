import json
import rclpy
from rclpy.node import Node
from .RRT.FB_RRT import IK_MDN_Model,MDNLayer,FB_RRTAngle,forward_kinematics_from_angles,get_ik
from .RRT.TangentRRT import IK_MDN_Model,MDNLayer, TangentPlanner
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
from moveit_msgs.msg import DisplayTrajectory, RobotTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math
from sensor_msgs.msg import LaserScan
from moveit_msgs.msg import PlanningScene, CollisionObject
from geometry_msgs.msg import Pose
import uuid
from shape_msgs.msg import SolidPrimitive
from rclpy.callback_groups import ReentrantCallbackGroup

from scipy.interpolate import CubicSpline
broker="172.23.66.117"
redis_server='172.23.248.33'

pool = redis.ConnectionPool(host=redis_server, port=6379, decode_responses=True, password='jimmy')
r = redis.Redis(connection_pool=pool)








class PathPlanningNode(Node):

    def __init__(self):
        super().__init__('path_planning')
              
        self.obstacles =[]# [0.7,0.1] np.load("/home/cotrobot/laser_scan/obstacles.npy")
        self.theta1, self.theta2, self.theta3 = 0, 0, 0
        self.joint_name = ["PrismaticJointZ", "Angle1", "Angle2", "Angle3", "GripperJoint1", "GripperJoint2", "GripperJoint3"]
        


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
        #jimmy add
        self.rrt_planning_pub = self.create_publisher(DisplayTrajectory, "/rrt_path", 10)
        #self.angle_rrt = FB_RRTAngle(tolerance=0.1, step_size=3.1415/180*5)
        #self.angle_rrt=TangentPlanner()
        #self.cb_group = ReentrantCallbackGroup()
        #self.sub_scan = self.create_subscription(LaserScan, "/scan", self.scan_callback, 10)

        #self.pub_scene = self.create_publisher(PlanningScene, "/planning_scene", 10)


    def update_move_status(self,msg):
        if msg.data=="Moving":
            self.robot_free=False
        else:
            self.robot_free=True



    def goal_callback(self, msg):
        self.get_logger().info(
            f"Received goal pose:\n"
            f"  Position: x={msg.pose.position.x}, y={msg.pose.position.y}\n"
            f"  Orientation: z={msg.pose.orientation.z}, w={msg.pose.orientation.w}"
        )

        # Find path in joint space (list of joint angles)
        #self.path = self.find_path_FB_RRT([msg.pose.position.x, msg.pose.position.y],[self.theta1, self.theta2, self.theta3])
        self.path = self.find_path([msg.pose.position.x, msg.pose.position.y],
                         [self.theta1, self.theta2, self.theta3])
        #self.path = self.find_path_TangentRRT([msg.pose.position.x, msg.pose.position.y])

        msg = DisplayTrajectory()
        traj = RobotTrajectory()
        jt = JointTrajectory()

        jt.joint_names = ["joint1", "joint2", "joint3"]
        if ((self.path is not None) and (len(self.path) > 0)):
            # 遍历 path 构建 trajectory
            for i, q in enumerate(self.path):
                #self.get_logger().info(f"q point:{q}")
                pt = JointTrajectoryPoint()
                pt.positions = [float(x) for x in q]
                pt.time_from_start.sec = i   # 简单设置每个点 1 秒
                jt.points.append(pt)

            traj.joint_trajectory = jt
            msg.trajectory = [traj]
            self.rrt_planning_pub.publish(msg)
            
            # Convert joint path to end-effector poses
            points_path = [forward_kinematics_from_angles(*angles)[-1] for angles in self.path]
            #print(points_path)
            self.get_logger().info(f"points_path: {points_path} ")
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
            r.set("mode","rrt_done")
            self.path=None

    def update_joint_states(self, msg):
        self.theta1, self.theta2, self.theta3 = msg.position[1], msg.position[2], msg.position[3]
    def find_path_TangentRRT(self,goal_xy): 
        # 3. 规划
        q_start = [self.theta1*3.14/180, self.theta2*3.14/180, (self.theta3-self.theta1-self.theta2)*3.14/180]
        data = r.get("obstacles")
        if data:
            self.obstacles= json.loads(data)
            print("Obstacles:", self.obstacles)
            #self.obstacles.clear()
            #r.set("obstacles",json.dumps(self.obstacles))

        path = self.angle_rrt.plan(q_start, goal_xy,self.obstacles,ik_samples=5000, tol=0.005,interp_steps=50)


        if path is None:
            print("❌ 没有找到路径")
            return

        print(f"\n✅ 找到路径，共 {len(path)} 个关节点")
        return path


    def find_path_FB_RRT(self,goal,start=[0,0,0]):
        # ------------ 你输入的是 WORLD 坐标 --------------
        #start_xy = (0.75, 0.05)
        goal_xy  = (0.1, 0.4)
            # ------------ 转成 joint angle（用 IK） --------------
        #q_start = get_ik(start_xy)
        #q_goal  = get_ik(goal_xy)
        angle = (self.theta3-self.theta1-self.theta2 )
        q_start=[self.theta1, self.theta2, angle]
        q_goal  =[3.14/2, 0, 0]

        print("Start q:", q_start)
        print("Goal  q:", q_goal)

            # ------------ 障碍物（圆形） --------------
        #obstacles = [
        #    (0.50, 0.18, 0.05),   # (cx, cy, r)
        #    #(0.15, 0.50, 0.07)
        #]
        data = r.get("obstacles")
        if data:
            self.obstacles= json.loads(data)
            print("Obstacles:", self.obstacles)
            #self.obstacles.clear()
            #r.set("obstacles",json.dumps(self.obstacles))
        path=self.angle_rrt.plan_path(q_start, q_goal, self.obstacles)
        print(path)
        return path

    def smooth_path(self, path, iterations=100):
        if path is None or len(path) < 3:
            return path  # 无需平滑

        path = [np.array(q, dtype=np.float32) for q in path]

        for _ in range(iterations):
            if len(path) <= 2:
                break

            # 随机选择两个节点 i < j
            i = np.random.randint(0, len(path) - 2)
            j = np.random.randint(i + 2, len(path))

            q1 = path[i]
            q2 = path[j]

            # 如果中间可直连 → 删除多余点
            if self.edge_is_collision_free(q1, q2):
                # 保留 i 和 j，中间全部替换成一条直线插值
                new_segment = [q1]

                steps = int(np.linalg.norm(q2 - q1) / self.step_size) + 2
                for a in np.linspace(0, 1, steps):
                    q = q1 + a * (q2 - q1)
                    new_segment.append(q)

                path = path[:i] + new_segment + path[j + 1:]

        return path


    def smooth_curve(self, path, points=30):
        """
        使用三次样条使路径更平滑（适合机械臂控制）
        :param path: 已 shortcut 的路径
        :param points: 生成多少个平滑点
        """
        path = np.array(path)

        if path.ndim == 1:
            if path.size == 0:
                raise ValueError("Path is empty, cannot smooth.")

            if path.size == 2:
                # 单点路径 → 复制一份形成两点，才能做 spline
                path = np.vstack([path, path])
            else:
                raise ValueError(f"Path format invalid: {path}")
            
        N = len(path)
        t = np.linspace(0, 1, N)

        # 每个关节一根 spline
        q1_spline = CubicSpline(t, path[:, 0])
        q2_spline = CubicSpline(t, path[:, 1])
        q3_spline = CubicSpline(t, path[:, 2])

        ts = np.linspace(0, 1, points)

        smooth_path = np.stack([
            q1_spline(ts),
            q2_spline(ts),
            q3_spline(ts)
        ], axis=1)
        return smooth_path
    
    def find_path(self,goal,start=[0, 0,0]):
        all_path=[]
        self.angle_rrt = RRTAngle(start,tolerance=0.1, step_size=3.1415/180*5)
        data = r.get("obstacles")
        if data:
            self.obstacles= json.loads(data)
            print("Obstacles:", self.obstacles)
            #self.obstacles.clear()
            #r.set("obstacles",json.dumps(self.obstacles))
        for x,y,rad in self.obstacles:
            self.angle_rrt.add_obstacle('circle', (x,y,rad))
        path1=self.angle_rrt.build_tree(goal,goal_bias=0.5,max_iterations=2000)

        if path1 is not None:
            all_path.extend(path1)
            self.angle_rrt = RRTAngle(path1[-1],tolerance=0.05, step_size=3.1415/180*2)
            for x,y,rad in self.obstacles:
                self.angle_rrt.add_obstacle('circle', (x,y,rad))
            path2=self.angle_rrt.build_tree(goal,goal_bias=0.5,max_iterations=1000)
        else:
            self.angle_rrt = RRTAngle(start,tolerance=0.1, step_size=3.1415/180*5)
            for x,y,rad in self.obstacles:
                self.angle_rrt.add_obstacle('circle', (x,y,rad))
            path2=self.angle_rrt.build_tree(goal,goal_bias=0.5,max_iterations=1000)
        
        
        if path2 is not None:
            all_path.extend(path2)
            self.angle_rrt = RRTAngle(path2[-1],tolerance=0.01, step_size=3.1415/180*2)
            for x,y,rad in self.obstacles:
                self.angle_rrt.add_obstacle('circle', (x,y,rad))
            path3=self.angle_rrt.build_tree(goal,goal_bias=1)
        else:
            path3=None

        if path3 is not None:
            all_path.extend(path3)

        #smooth2 = self.smooth_curve(all_path, points=20)
        #print("曲线平滑后节点:", len(smooth2))
        print("all_path:", all_path)

        return all_path
    
# ✅ 创建圆柱体障碍物
    def addobject(self,ps,x,y):
 

        co = CollisionObject()
        co.id = "pillar_" + str(uuid.uuid4())[:8]
        co.header.frame_id = "base_link"
        co.operation = CollisionObject.ADD
        cylinder = SolidPrimitive()
        cylinder.type = SolidPrimitive.CYLINDER
        cylinder.dimensions = [0.5, 0.]  # 高0.5m 半径2cm

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
        self.column_count=0
        self.obstacles.clear()
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
        #time.sleep(100)

from rclpy.executors import MultiThreadedExecutor

def main(args=None):
    #rclpy.init(args=args)
    #path_planner = PathPlanningNode()
    #rclpy.spin(path_planner)
    #path_planner.destroy_node()
    #rclpy.shutdown()

    rclpy.init(args=args)
    path_planner = PathPlanningNode()
    # 使用多线程执行器
    executor = MultiThreadedExecutor()
    executor.add_node(path_planner)
    try:
        executor.spin()
    finally:
        path_planner.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()
