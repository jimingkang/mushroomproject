import rclpy
from rclpy.node import Node
import pymoveit2

def main():
    rclpy.init()
    node = Node("pymoveit2_example")

    # ??? PyMoveIt2 ??(???,???? move_group)
    moveit2 = pymoveit2.MoveIt2(
        node=node,
        joint_names=[
            "joint1", "joint2", "joint3", "joint4"   # ????????
        ],
        base_link_name="base_link",   # ?????
        end_effector_name="link5",    # ?????
        group_name="scara_arm",             # MoveIt2 ???? planning_group
    )

    # ??:??????
    #moveit2.move_to_configuration([0.0, 1.57, 1.57, 0.0]
    moveit2.planner_id="PTP"
    moveit2.max_velocity = 0.5
    moveit2.max_acceleration = 0.5
    # ?????????
    moveit2.move_to_pose(
        position=[0.50, 0.0, 0.1],   # x,y,z
        quat_xyzw=[0.0, 0.0, 0.0, 1.0] , # ???
        #planner_id="PTP"
    )

    rclpy.shutdown()

if __name__ == "__main__":
    main()

