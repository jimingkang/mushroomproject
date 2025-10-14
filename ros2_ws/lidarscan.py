#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from moveit_msgs.msg import PlanningScene, CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
import math

class LidarWallPublisher(Node):
    def __init__(self):
        super().__init__("lidar_wall_node")
        self.sub_scan = self.create_subscription(LaserScan, "/scan", self.scan_callback, 10)
        self.pub_scene = self.create_publisher(PlanningScene, "/planning_scene", 10)
        self.get_logger().info("✅ 激光墙体障碍物节点启动")

    def scan_callback(self, msg: LaserScan):
        points = []
        angle = msg.angle_min
        self.get_logger().info("✅ scan_callback")
        for r in msg.ranges:
            if msg.range_min < r < msg.range_max:
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                points.append((x, y))
            angle += msg.angle_increment

        if len(points) < 2:
            return

        # ✅ 简化方法：找出扫描到的最近和最远点来构成一条线段
        points.sort(key=lambda p: math.sqrt(p[0]**2 + p[1]**2))
        p1 = points[0]
        p2 = points[-1]

        x1, y1 = p1
        x2, y2 = p2

        # ✅ 墙的参数计算
        cx = (x1 + x2) / 2.0
        cy = (y1 + y2) / 2.0
        cz = 0.5  # 墙的高度中心（比如 1m 高的墙）

        length = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        thickness = 0.05  # 墙体厚度
        height = 1.0      # 墙体高度（你可以改）

        yaw = math.atan2(y2 - y1, x2 - x1)
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)

        # ✅ 构建 CollisionObject
        ps = PlanningScene()
        ps.is_diff = True

        co = CollisionObject()
        co.id = "scan_wall"
        co.header.frame_id = "world"
        co.operation = CollisionObject.ADD

        wall = SolidPrimitive()
        wall.type = SolidPrimitive.BOX
        wall.dimensions = [length, thickness, height]  # 长、宽、高

        pose = Pose()
        pose.position.x = cx
        pose.position.y = cy
        pose.position.z = cz
        pose.orientation.z = qz
        pose.orientation.w = qw

        co.primitives.append(wall)
        co.primitive_poses.append(pose)

        ps.world.collision_objects.append(co)
        self.pub_scene.publish(ps)

        self.get_logger().info(
            f"✅ 已发布墙体障碍物：长度={round(length,2)}m，位置=({round(cx,2)}, {round(cy,2)}), yaw={round(yaw,2)}rad"
        )

def main():
    rclpy.init()
    node = LidarWallPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
