from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


#def generate_launch_description():
#    moveit_config = MoveItConfigsBuilder("scara_description", package_name="scara_moveit_config").kinematics(file_path="config/kinematics.yaml")
    
#    return generate_demo_launch(moveit_config)

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("scara_description", package_name="scara_moveit_config")
        .robot_description(file_path="config/scara_description.urdf.xacro")  # Load URDF
        .robot_description_semantic(file_path="config/scara_description.srdf")  # Load SRDF
        .trajectory_execution(file_path="config/moveit_controllers.yaml")  # Controllers
        .planning_pipelines(file_path="config/ompl_planning.yaml")  # OMPL Config
        .joint_limits(file_path="config/joint_limits.yaml")  # Joint Limits
        .to_moveit_configs()
    )

    # ✅ Manually Load kinematics.yaml
    kinematics_yaml = os.path.join(
        get_package_share_directory("scara_moveit_config"),
        "config",
        "kinematics.yaml"
    )

    # ✅ Move Group Node (Loads kinematics.yaml manually)
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), kinematics_yaml],  # Manually adding kinematics.yaml
    )

    return LaunchDescription([move_group_node])

