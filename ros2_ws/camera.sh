source ~/.bashrc
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=5
ros2 launch realsense2_camera rs_launch.py     rgb_camera.profile:=848x480x15     depth_module.profile:=848x480x15     align_depth.enable:=true
