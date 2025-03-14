import launch
import launch.actions
import launch.substitutions
import launch_ros.actions

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
    #    Node(
    #        package='realsense_ros2',
    #        node_executable='rs_t265_node',
    #        node_name='rs_t265',
    #        output='screen'
    #    ),
        Node(
            package='realsense_ros2',
            executable='rs_d435_node',
            name='rs_d435',
            output='screen',
            parameters=[
                {"publish_depth": True},
                {"publish_pointcloud": True},
                {"is_color": True},
                {"publish_image_raw_": True},
                {"fps": 30}      # Can only take values of 6,15,30 or 60
            ]
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'world', 'map']
            ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0',  'world','odom']
            ),
        Node(
            # Configure the TF of the robot 
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0.0', '0.6', '0.0',  '1.57', '-1.57','0.0', 'world', 'base_link']
        ),
        Node(
            # Configure the TF of the robot 
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'base_link', 'camera_link_d435']
        ),
        #Node(
        #    # Configure the TF of the robot 
        #    package='tf2_ros',
        #    executable='static_transform_publisher',
        #    output='screen',
        #    arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'odom', 'base_link']
        #),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0.0', '0.0', '0.6', '-1.57','0.0', '-1.57', 'world', 'camera_link_d435']
            ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
         #   arguments=['0.0', '0.025', '0.03', '-1.5708', '0.0', '-1.5708', 'base_link', 'camera_d435_link']
            arguments=['0.0', '0.0', '0.6', '-1.57', '0.0', '-1.57', 'base_link', 'camera_link_d435_pcl']

            ),
            
         Node(
             package='depthimage_to_laserscan',
             executable='depthimage_to_laserscan_node',
             name='scan',
             output='screen',
             parameters=[{'output_frame':'camera_d435_link'}],
             remappings=[('depth','rs_d435/aligned_depth/image_raw'),
                         ('depth_camera_info', 'rs_d435/aligned_depth/camera_info')],
             ),
    ])
