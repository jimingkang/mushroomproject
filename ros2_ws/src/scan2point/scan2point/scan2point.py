#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
import laser_geometry.laser_geometry as lg

class ScanToCloud(Node):
    def __init__(self):
        super().__init__('scan_to_cloud_node')
        self.lp = lg.LaserProjection()
        self.sub = self.create_subscription(
            LaserScan,
            '/scan',   # 订阅 RPLIDAR 的 scan
            self.scan_callback,
            10
        )
        self.pub = self.create_publisher(
            PointCloud2,
            '/scan_cloud',  # 发布转换后的点云
            10
        )
        self.get_logger().info("✅ scan_to_cloud 节点已启动，正在监听 /scan")

    def scan_callback(self, msg):
        cloud = self.lp.projectLaser(msg)
        cloud.header = msg.header
        self.pub.publish(cloud)

def main(args=None):
    rclpy.init(args=args)
    node = ScanToCloud()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

