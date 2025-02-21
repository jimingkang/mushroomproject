from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch

from launch import LaunchDescription
from launch_ros.actions import Node

#def generate_launch_description():
#    moveit_config = MoveItConfigsBuilder("scara_description", package_name="scara_moveit_config").to_moveit_configs()
#    return generate_demo_launch(moveit_config)
def generate_launch_description():
    # Load MoveIt configuration
    moveit_config = (
        MoveItConfigsBuilder("scara_description", package_name="scara_moveit_config")
        .robot_description(file_path="config/scara_description.urdf.xacro")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .sensors_3d(file_path="config/sensor3D.yaml")  # Load the 3D sensor configuration
        .to_moveit_configs()
    )

    # Start the MoveIt demo
    demo_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    return LaunchDescription([demo_node])
