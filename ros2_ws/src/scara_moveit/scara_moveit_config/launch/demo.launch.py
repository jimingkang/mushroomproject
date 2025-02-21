from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os
#def generate_launch_description():
#    moveit_config = MoveItConfigsBuilder("scara_description", package_name="scara_moveit_config").to_moveit_configs()
#    return generate_demo_launch(moveit_config)
def generate_launch_description():
    # Load MoveIt configuration
    moveit_config = (
        MoveItConfigsBuilder("scara_description", package_name="scara_moveit_config")
        .robot_description(file_path="config/scara_description.urdf.xacro")
        .robot_description_semantic(file_path="config/scara_description.srdf")
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
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d",  get_package_share_directory("scara_moveit_config") + "/config/moveit.rviz"],  # Path to RViz config
        #arguments=["-d", str(moveit_config.package_path / "config/moveit.rviz")],  # Path to RViz config
        parameters=[moveit_config.robot_description,moveit_config.robot_description_semantic],
    )
    ros2_controllers_path = os.path.join(
        get_package_share_directory("scara_moveit_config"),
        "config",
        "ros2_controllers.yaml",
        )

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[moveit_config.robot_description,ros2_controllers_path],
    )
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager-timeout",
            "300",
            "--controller-manager",
            "/controller_manager",
        ],
    )
    scara_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["scara_arm_controller", "-c", "/controller_manager"],
    )
    robot_state_publisher_node = Node(
    package="robot_state_publisher",
    executable="robot_state_publisher",
    name="robot_state_publisher",
    output="screen",
    parameters=[moveit_config.robot_description],
    )
    # Return the launch description
    return LaunchDescription([
        demo_node,
        rviz_node,
        controller_manager_node,
        joint_state_broadcaster_spawner,
        scara_arm_controller_spawner,
        robot_state_publisher_node,
    ])
    #return LaunchDescription([demo_node])
