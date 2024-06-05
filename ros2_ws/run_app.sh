cd ~/Downloads/mushroomproject/ros2_ws/src/YOLOX-ROS/flask_ros_app
cp -fr   flask_ros_app/ /home/jimmy/Downloads/mushroomproject/ros2_ws/install/flask_ros_app/lib/python3.8/site-packages/
cd ~/Downloads/mushroomproject/ros2_ws/
source install/setup.bash;ros2 run flask_ros_app flask_ros_app
