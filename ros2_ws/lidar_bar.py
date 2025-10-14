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

        for r in msg.ranges:
            if msg.range_min < r < msg.range_max:
                # ✅ 将极坐标转换为笛卡尔坐标
                x = r * math.cos(angle)
                y = r * math.sin(angle)

                # ✅ 过滤掉太近或太远的点（比如 5cm ~ 2m）
                dist = math.sqrt(x**2 + y**2)
                if dist < 0.05 or dist > 2.0:
                    angle += msg.angle_increment
                    continue

                # ✅ 创建一个圆柱体障碍物
                co = CollisionObject()
                co.id = "pillar_" + str(uuid.uuid4())[:8]  # 给每个立柱一个唯一ID
                co.header.frame_id = "base_link"
                co.operation = CollisionObject.ADD

                cylinder = SolidPrimitive()
                cylinder.type = SolidPrimitive.CYLINDER
                cylinder.dimensions = [0.5, 0.02]  # 高度0.5m，半径2cm 的小立柱

                pose = Pose()
                pose.position.x = x
                pose.position.y = y
                pose.position.z = 0.25  # 立柱中心点高度（0.5m 高的立柱中心是 0.25m）
                pose.orientation.w = 1.0

                co.primitives.append(cylinder)
                co.primitive_poses.append(pose)

                ps.world.collision_objects.append(co)
                column_count += 1

            angle += msg.angle_increment

        # ✅ 发布所有立柱到规划场景
        if column_count > 0:
            self.pub_scene.publish(ps)
            self.get_logger().info(f"📡 已发布 {column_count} 个立柱障碍物")
        time.sleep(10)

def main():
    rclpy.init()
    node = ScanToCylinders()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

