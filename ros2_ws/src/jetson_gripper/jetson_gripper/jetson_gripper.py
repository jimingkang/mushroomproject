#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

import board
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo


class ServoControllerNode(Node):
    def __init__(self):
        super().__init__('jetson_gripper_node')

        # --- 初始化 I2C 和 PCA9685 ---
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(self.i2c)
        self.pca.frequency = 60  # 舵机常用频率 50~60Hz

        # --- 初始化舵机通道 ---
        self.servo1 = servo.Servo(self.pca.channels[0])
        self.servo2 = servo.Servo(self.pca.channels[1])
        self.servo3 = servo.Servo(self.pca.channels[2])
        self.servo_yaw = servo.Servo(self.pca.channels[3])

        self.get_logger().info("✅ PCA9685 init ,pwm: 60Hz")
        self.get_logger().info("✅ Servo channels: 0, 1, 3, 4")

        # --- 创建 ROS2 Service ---
        self.srv_gripper_close = self.create_service(
            Trigger, 'close_gripper_service', self.close_gripper_service_callback)
        self.srv_gripper_open = self.create_service(
            Trigger, 'open_gripper_service', self.open_gripper_service_callback)

        self.get_logger().info("✅  close_gripper_service / open_gripper_service")

    # --- 抓手关闭 ---
    def close_gripper_service_callback(self, request, response):
        try:
            self.servo1.angle = 130
            self.servo2.angle = 130
            self.servo3.angle = 130
            self.get_logger().info("🤖 抓手关闭")
            response.success = True
            response.message = "Gripper closed successfully."
        except Exception as e:
            response.success = False
            response.message = f"Error closing gripper: {e}"
        return response

    # --- 抓手打开 ---
    def open_gripper_service_callback(self, request, response):
        try:
            self.servo1.angle = 80
            self.servo2.angle = 80
            self.servo3.angle = 80
            self.get_logger().info("🖐️ open")
            response.success = True
            response.message = "Gripper opened successfully."
        except Exception as e:
            response.success = False
            response.message = f"Error opening gripper: {e}"
        return response

    def destroy_node(self):
        self.pca.deinit()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ServoControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("shudown servo controller node")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
