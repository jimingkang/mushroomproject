from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os
def generate_launch_description():
#    moveit_config = MoveItConfigsBuilder("scara_description", package_name="scara_moveit_config").to_moveit_configs()
    moveit_config = (
        MoveItConfigsBuilder("scara_description", package_name="scara_moveit_config")
        .robot_description(file_path="config/scara_description.urdf.xacro")
        .robot_description_semantic(file_path="config/scara_description.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
       # .sensors_3d(file_path="config/sensor_3d.yaml")  # Load the 3D sensor configuration
        .to_moveit_configs()
        )
    return generate_demo_launch(moveit_config)
