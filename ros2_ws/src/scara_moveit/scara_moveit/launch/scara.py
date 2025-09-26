from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import os

def generate_launch_description():
    # ? 1. ?? moveit_config
    moveit_config = (
        MoveItConfigsBuilder("scara_description", package_name="scara_moveit_config")
        .robot_description(file_path=os.path.join(get_package_share_directory("scara_moveit_config"), "config", "scara_description.urdf.xacro"))
        .robot_description_semantic(file_path=os.path.join(get_package_share_directory("scara_moveit_config"), "config", "scara_description.srdf"))
        .trajectory_execution(file_path=os.path.join(get_package_share_directory("scara_moveit_config"), "config", "moveit_controllers.yaml"))
        .planning_pipelines(pipelines=["ompl"])
        .joint_limits(file_path=os.path.join(get_package_share_directory("scara_moveit_config"), "config", "joint_limits.yaml"))
        .to_moveit_configs()
    )

    # ? 2. include demo.launch.py ?????
    moveit_demo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("scara_moveit_config"),
                "launch/demo.launch.py",
            )
        ),
        launch_arguments=moveit_config.to_dict().items() if moveit_config.to_dict() else {} # ?? ??:? moveit_config ?????
    )

    ld = LaunchDescription()
    ld.add_action(moveit_demo_launch)
    return ld

