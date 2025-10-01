#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import PlanningScene, CollisionObject
from shape_msgs.msg import SolidPrimitive
import pymoveit2 


class MotionPlanner(Node):
    def __init__(self):
        super().__init__("motion_planner")

        # ✅ 初始化 MoveIt2
        self.moveit2 = pymoveit2.MoveIt2(
            node=self,
            joint_names=["joint1", "joint2", "joint3", "joint4"],
            base_link_name="base_link",
            end_effector_name="link5",
            group_name="scara_arm"
        )
        self.moveit2.planner_id = "PTP"
        self.moveit2.max_velocity = 0.5
        self.moveit2.max_acceleration = 0.5

        # ✅ PlanningScene 发布器
        self._pub_ps = self.create_publisher(PlanningScene, "/monitored_planning_scene", 10)

        # ✅ 启动 1 秒后添加障碍物，3 秒后开始规划
        self.create_timer(1.0, self.add_cylinder_once)
        self.create_timer(3.0, self.plan_and_print_once)

        self.obstacle_added = False
        self.planned_once = False

    def add_cylinder_once(self):
        if self.obstacle_added:
            return
        self.obstacle_added = True

        ps = PlanningScene()
        ps.is_diff = True

        # 定义一个圆柱障碍物
        co = CollisionObject()
        co.id = "obstacle1"
        co.header.frame_id = "world"
        co.operation = CollisionObject.ADD

        cylinder = SolidPrimitive()
        cylinder.type = SolidPrimitive.CYLINDER
        cylinder.dimensions = [0.8, 0.08]  # 高度=0.4m, 半径=0.08m

        pose = PoseStamped()
        pose.header.frame_id = "world"
        pose.pose.position.x = 0.35  # 放在机械臂前方 35cm
        pose.pose.position.y = 0.1
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0

        co.primitives.append(cylinder)
        co.primitive_poses.append(pose.pose)

        ps.world.collision_objects.append(co)
        self._pub_ps.publish(ps)

        self.get_logger().info("✅ 已添加圆柱障碍物 obstacle1")

    def plan_and_print_once(self):
        if self.planned_once:
            return
        self.planned_once = True

        self.get_logger().info("📡 开始规划路径（考虑障碍物）...")

        traj = self.moveit2.move_to_pose(
            position=[0.,-0.6, 0.10],   # 目标点在障碍物后方
            quat_xyzw=[0.0, 0.0, 0.0, 1.0]
        )

        if traj is None:
            self.get_logger().error("❌ 规划失败，可能无法绕过障碍物")
            return

        self.get_logger().info(f"✅ 规划成功，轨迹点数: {len(traj.joint_trajectory.points)}")
        for i, point in enumerate(traj.joint_trajectory.points):
            positions = [round(p, 3) for p in point.positions]
            self.get_logger().info(f"  点 {i}: {positions}")

        # ✅ 执行轨迹（如不想执行可注释掉）
        #self.moveit2.execute(traj)


def main():
    rclpy.init()
    node = MotionPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

