#!/usr/bin/env python3
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

class ScanToCylinders(Node):
    def __init__(self):
        super().__init__("scan_to_cylinders")
        self.sub_scan = self.create_subscription(LaserScan, "/scan", self.scan_callback, 10)
        self.pub_scene = self.create_publisher(PlanningScene, "/planning_scene", 10)
        self.get_logger().info("✅ 已启动: 将 /scan 转换为 3D 细小立柱障碍物")

    def scan_callback(self, msg: LaserScan):
        ps = PlanningScene()
        ps.is_diff = True

        angle = msg.angle_min
        column_count = 0
        co = CollisionObject()
        co.id = "all"                     # ID 可随意，只要唯一即可
        co.operation = CollisionObject.REMOVE
        ps.world.collision_objects.append(co)
        self.pub_scene.publish(ps)

        angle = msg.angle_min
        layer_points = []
        for r in msg.ranges:
            if msg.range_min < r < msg.range_max:
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                if 0.05 < math.sqrt(x**2 + y**2) < 2.0:
                    layer_points.append((x, y))
            angle += msg.angle_increment

        # 聚类
        clusters = []
        cluster = [layer_points[0]]
        for i in range(1, len(layer_points)):
            prev = layer_points[i-1]
            curr = layer_points[i]
            d = math.hypot(curr[0]-prev[0], curr[1]-prev[1])
            if d < 0.01:
                cluster.append(curr)
            else:
                #if len(cluster) >= 10:
                clusters.append(cluster)
                cluster = [curr]
        #if len(cluster) >= 10:
        #    clusters.append(cluster)

        # 分类并生成障碍物
        for cluster in clusters:
            if len(cluster) < 10:
                # 小簇 → 立柱
                for (x, y) in cluster:
                    co = CollisionObject()
                    co.id = "pillar_" + str(uuid.uuid4())[:8]
                    co.header.frame_id = "base_link"
                    co.operation = CollisionObject.ADD

                    cylinder = SolidPrimitive()
                    cylinder.type = SolidPrimitive.CYLINDER
                    cylinder.dimensions = [0.5, 0.01]

                    pose = Pose()
                    pose.position.x = x
                    pose.position.y = y
                    pose.position.z = 0.25
                    pose.orientation.w = 1.0

                    co.primitives.append(cylinder)
                    co.primitive_poses.append(pose)
                    ps.world.collision_objects.append(co)

            else:
                # 多点簇 → 横梁，拟合线段
                pts = np.array(cluster)
                x_min, y_min = pts.min(axis=0)
                x_max, y_max = pts.max(axis=0)

                # 拟合中心与朝向
                center_x = (x_min + x_max) / 2
                center_y = (y_min + y_max) / 2
                length = math.hypot(x_max - x_min, y_max - y_min)
                yaw = math.atan2(y_max - y_min, x_max - x_min)

                co = CollisionObject()
                co.id = "beam_" + str(uuid.uuid4())[:8]
                co.header.frame_id = "base_link"
                co.operation = CollisionObject.ADD

                box = SolidPrimitive()
                box.type = SolidPrimitive.BOX
                box.dimensions = [length, 0.5, 0.05]  # 长、宽、高

                pose = Pose()
                pose.position.x = center_x
                pose.position.y = center_y
                pose.position.z = 0.25  # 平层中心高度
                pose.orientation.z = math.sin(yaw / 2)
                pose.orientation.w = math.cos(yaw / 2)

                co.primitives.append(box)
                co.primitive_poses.append(pose)
                ps.world.collision_objects.append(co)
                column_count += 1

        # ✅ 发布所有立柱到规划场景
        if column_count >=0:
            self.pub_scene.publish(ps)
            self.get_logger().info(f"📡 已发布 {column_count} 个横梁障碍物")
        time.sleep(5)

def main():
    rclpy.init()
    node = ScanToCylinders()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

