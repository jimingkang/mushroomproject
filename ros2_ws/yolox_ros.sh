cd ~/Documents/mushroomproject/ros2_ws/src/YOLOX-ROS/yolox_ros_py
cp -fr   yolox_ros_py/ ~/Documents/mushroomproject/ros2_ws/install/yolox_ros_py/lib/python3.8/site-packages/
cd ~/Documents/mushroomproject/ros2_ws/
source install/setup.bash
ros2 launch yolox_ros_py yolox_nano_torch_gpu_camera.launch.py
