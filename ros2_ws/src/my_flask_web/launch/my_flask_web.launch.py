import launch
import launch_ros.actions
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
 
    my_flask_web_node = launch_ros.actions.Node(
        package="my_flask_web", executable="my_flask_web_node",
        #parameters=[
        #    {"yolox_exp_py" : yolox_ros_share_dir+'/yolox_nano.py'},
        #    {"device" : 'gpu'},
        #    {"fp16" : True},
        #    {"fuse" : False},
        #    {"legacy" : False},
        #    {"trt" : True},
        #    {"ckpt" : yolox_ros_share_dir+"/yolox_nano.pth"},  # only useful for no trt
        #    {"conf" : 0.3},
        #    {"threshold" : 0.45},
        #    {"resize" : 640},
       # ],
    )


    return launch.LaunchDescription([
        my_flask_web_node,

    ])