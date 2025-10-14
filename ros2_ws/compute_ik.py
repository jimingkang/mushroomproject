import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK

class IKClient(Node):
    def __init__(self):
        super().__init__('ik_client')
        self.cli = self.create_client(GetPositionIK, '/compute_ik')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /compute_ik service...')

        self.req = GetPositionIK.Request()

    def send_request(self):
        # 填写请求数据
        self.req.ik_request.group_name = 'scara_arm'  # 改成你的 MoveIt group
        self.req.ik_request.avoid_collisions = False
        #self.req.ik_request.attempts = 5

        # 目标姿态
        pose = PoseStamped()
        pose.header.frame_id = 'base_link'
        pose.pose.position.x = 0.5
        pose.pose.position.y = 0.0
        pose.pose.position.z = 0.1
        pose.pose.orientation.w = 1.0
        self.req.ik_request.pose_stamped = pose

        # 调用服务
        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

def main():
    rclpy.init()
    node = IKClient()
    response = node.send_request()

    if response.error_code.val == response.error_code.SUCCESS:
        joint_names = response.solution.joint_state.name
        joint_positions = response.solution.joint_state.position  # 转换为角度
        joint_positions = [pos * 180 / math.pi for pos in joint_positions]
        node.get_logger().info('✅ IK Computed Successfully!')
        for name, pos in zip(joint_names, joint_positions):
            node.get_logger().info(f'{name}: {pos:.4f}')
    else:
        node.get_logger().error(f'❌ IK computation failed: code={response.error_code.val}')

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
