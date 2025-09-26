from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # ??????
    moveit_config = (
        MoveItConfigsBuilder("scara_description", package_name="scara_moveit_config")
        .to_moveit_configs()
    )

    # ? ?????????? Pilz,??? OMPL
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"planning_pipelines": ["pilz_industrial_motion_planner"]},
            {"default_planning_pipeline": "pilz_industrial_motion_planner"},  # ???
        ],
    )

    return LaunchDescription([move_group_node])

