import math
import time
import uuid
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import PlanningScene, CollisionObject

class ScanToCylinders(Node):
    def __init__(self):
        super().__init__("scan_to_cylinders")
        self.sub_scan = self.create_subscription(LaserScan, "/scan", self.scan_callback, 10)
        self.pub_scene = self.create_publisher(PlanningScene, "/planning_scene", 10)
        self.get_logger().info("✅ 已启动: 将 /scan 转换为 3D 细小立柱障碍物，并查找最近障碍物")

    def scan_callback(self, msg: LaserScan):
        ps = PlanningScene()
        ps.is_diff = True

        angle = msg.angle_min
        column_count = 0

        # 📌 添加：最近障碍物追踪
        min_dist = float("inf")
        nearest_bar_pose = None

        for r in msg.ranges:
            if msg.range_min < r < msg.range_max:
                # ✅ 极坐标 → 笛卡尔坐标
                x = r * math.cos(angle)
                y = r * math.sin(angle)

                # ✅ 距离过滤（可选）
                dist = math.sqrt(x**2 + y**2)
                if dist < 0.05 or dist > 2.0:
                    angle += msg.angle_increment
                    continue

                # ✅ 创建圆柱体障碍物
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
                column_count += 1

                # 📍 最近障碍物更新逻辑
                if dist < min_dist:
                    min_dist = dist
                    nearest_bar_pose = pose

            angle += msg.angle_increment

        # ✅ 发布所有立柱障碍物
        if column_count > 0:
            self.pub_scene.publish(ps)
            self.get_logger().info(f"📡 已发布 {column_count} 个立柱障碍物")

            # 📍 输出最近立柱信息
            if nearest_bar_pose is not None:
                self.get_logger().info(
                    f"📍 最近的障碍柱：距离 {min_dist:.3f} m，位置 "
                    f"({nearest_bar_pose.position.x:.3f}, {nearest_bar_pose.position.y:.3f}, {nearest_bar_pose.position.z:.3f})"
                )

        time.sleep(10)
