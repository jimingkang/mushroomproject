from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("scara_description", package_name="scara_moveit_config").kinematics(file_path="config/kinematics.yaml")
    return generate_demo_launch(moveit_config)
