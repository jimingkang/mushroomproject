import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import ExecuteProcess
import xacro
from moveit_configs_utils import MoveItConfigsBuilder

from launch.substitutions import Command

from launch_ros.substitutions import FindPackageShare


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def loadyaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None

def generate_launch_description():
    moveit_config = 	MoveItConfigsBuilder("scara_description",package_name="scara_moveit_config").to_moveit_configs()

    # Get parameters for the Servo node
    servo_yaml = loadyaml("scara_servo", "config/servo_config.yaml")
    print(servo_yaml)
    servo_params = {"moveit_servo": servo_yaml}

    # RViz
    rviz_config_file = (
        get_package_share_directory("scara_moveit_config") + "/config/moveit.rviz"
    )
        # Robot Description
    robot_description_config = os.path.join(
        get_package_share_directory('scara_moveit_config'),
        'config',
        'scara_description.urdf.xacro'
    )
    robot_description = {'robot_description': Command(['xacro ', robot_description_config])}

    # Semantic Robot Description
    robot_description_semantic_config = os.path.join(
        get_package_share_directory('scara_moveit_config'),
        'config',
        'scara_description.srdf'
    )
    robot_description_semantic = {
        'robot_description_semantic': Command(['xacro ', robot_description_semantic_config])
    }
    
    # Kinematics
    kinematics_config = os.path.join(
        get_package_share_directory('scara_moveit_config'),
        'config',
        'kinematics.yaml'
    )

    # Planning Pipeline
    planning_pipeline_config = os.path.join(
        get_package_share_directory('scara_moveit_config'),
        'config',
        'ompl_planning.yaml'
    )

    # MoveIt Controllers
    moveit_controllers_config = os.path.join(
        get_package_share_directory('scara_moveit_config'),
        'config',
        'moveit_controllers.yaml'
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            #moveit_config.robot_description,
            #moveit_config.robot_description_semantic,
            robot_description,
            robot_description_semantic,
        ],
    )

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("scara_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
        #moveit_config.robot_description, 
        robot_description, 
        ros2_controllers_path
        ],
        output="screen",
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


    # Move Group Node
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_config,
            planning_pipeline_config,
            moveit_controllers_config
        ]
    )

    servo_node = Node(
        package="scara_servo",
        executable="scara_servo",
        name="scara_servo",
        parameters=[
            servo_params,
            #moveit_config.robot_description,
            #moveit_config.robot_description_semantic,
            #moveit_config.robot_description_kinematics,
            robot_description,
            robot_description_semantic,
            #robot_description_kinematics,
            kinematics_config,
            #joint_limits,
        ],
        output="screen",
    )

    # Gazebo nodes
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory("gazebo_ros"), "/launch", "/gazebo.launch.py"]
        ),
    )

    # Spawn robot "-topic", "robot_description"
    gazebo_spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_scara",
        arguments=["-entity", "scara_description","-topic", "/robot_description"],
        output="screen",
    )
    moveit_demo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('scara_moveit_config'),
                        'launch/demo.launch.py')
        )
    )

    ld = LaunchDescription([
    ])
    #ld.add_action(rviz_node)
    #ld.add_action(gazebo)
    #ld.add_action(gazebo_spawn_robot)

    #ld.add_action(servo_node)
    #ld.add_action(ros2_control_node)
    #ld.add_action(joint_state_broadcaster_spawner)
    #ld.add_action(scara_arm_controller_spawner)
    #ld.add_action(move_group_node)

    ld.add_action(moveit_demo_launch)

    return ld
   
    
