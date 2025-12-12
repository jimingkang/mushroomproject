import json
import rclpy
from rclpy.node import Node
#from .RRT.FB_RRT import IK_MDN_Model,MDNLayer,FB_RRTAngle,forward_kinematics_from_angles,get_ik
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







class PublishObsNode(Node):

    def __init__(self):
        super().__init__('PublishObsNode')
              
        self.obstacles =[]# [0.7,0.1] np.load("/home/cotrobot/laser_scan/obstacles.npy")
        self.column_count=0
        #jimmy add


        #self.cb_group = ReentrantCallbackGroup()
        self.sub_scan = self.create_subscription(LaserScan, "/scan", self.scan_callback, 10)

        self.pub_scene = self.create_publisher(PlanningScene, "/planning_scene", 10)




# ✅ 创建圆柱体障碍物
    def addobject(self,ps,x,y):
        co = CollisionObject()
        co.id = "pillar_" + str(uuid.uuid4())[:8]
        co.header.frame_id = "base_link"
        co.operation = CollisionObject.ADD
        cylinder = SolidPrimitive()
        cylinder.type = SolidPrimitive.CYLINDER
        cylinder.dimensions = [0.5, 0.08]  # 高0.5m 半径2cm

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
                self.addobject(ps,x,y)
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
            self.obstacles.append((ox, oy,0.1))


        self.get_logger().info(f"检测到 {len(self.obstacles)} 个障碍物")
        for i, (cx, cy,rad) in enumerate(self.obstacles):
            self.addobject(ps, cx, cy)
            self.get_logger().info(f"障碍物 {i+1}: x={cx:.2f}, y={cy:.2f}")

        # ✅ 发布所有立柱到规划场景
        if self.column_count > 0:
            self.pub_scene.publish(ps)
            self.get_logger().info(f"📡 已发布 {self.column_count} 个立柱障碍物")
            r.set("obstacles", json.dumps(self.obstacles))
        time.sleep(10)

from rclpy.executors import MultiThreadedExecutor

def main(args=None):
    #rclpy.init(args=args)
    #path_planner = PathPlanningNode()
    #rclpy.spin(path_planner)
    #path_planner.destroy_node()
    #rclpy.shutdown()

    rclpy.init(args=args)
    path_planner = PublishObsNode()
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
