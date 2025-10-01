from pymoveit2 import PlanningSceneInterface
from geometry_msgs.msg import PoseStamped
import rclpy

rclpy.init()

# 初始化接口（需要 ROS2 节点上下文）
scene = PlanningSceneInterface(node_name="scene_interface")

# 定义障碍物的位姿
pose = PoseStamped()
pose.header.frame_id = "base_link"
pose.pose.position.x = 0.5
pose.pose.position.y = 0.0
pose.pose.position.z = 0.25   # 物体中心位置
pose.pose.orientation.w = 1.0

# 添加一个立方体（0.2m 边长）
scene.add_box("box1", pose, (0.2, 0.2, 0.2))

# 添加一个圆柱体（高 0.4m，半径 0.1m）
scene.add_cylinder("cylinder1", pose, height=0.4, radius=0.1)

# 查看已知物体
print("Current objects:", scene.get_known_object_names())

