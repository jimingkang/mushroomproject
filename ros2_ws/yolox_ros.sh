cd /home/jimmy/Downloads/mushroomproject/ros2_ws/src/YOLOX-ROS/yolox_ros_py
cp -fr   yolox_ros_py/ /home/jimmy/Downloads/mushroomproject/ros2_ws/install/yolox_ros_py/lib/python3.10/site-packages/
cd /home/jimmy/Downloads/mushroomproject/ros2_ws/
source install/setup.bash
#nohup ros2 launch yolox_ros_py yolox_nano_torch_gpu_camera.launch.py > yolox_ros.log &
 ros2 launch yolox_ros_py yolox_nano_torch_gpu_camera.launch.py  camera_name:=d405
