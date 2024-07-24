# install_ros2.sh
#!/bin/bash

# Install ROS2 base and development tools
apt install ros-humble-ros-base -y
apt install ros-dev-tools -y

# Configure shell to source ROS2 setup
# echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc (optional)
echo 'source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash' >> ~/.bashrc

#install librealsense
sudo mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null
echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | sudo tee /etc/apt/sources.list.d/librealsense.list
sudo apt-get update
sudo apt-get install librealsense2-dkms
sudo apt-get install librealsense2-utils
sudo apt install build-essential libboost-system-dev libboost-thread-dev libboost-program-options-dev libboost-test-dev
sudo apt install libpython3-dev libboost-python-dev
sudo apt-get install python3-opencv
sudo apt-get install libopencv-dev python3-opencv libopencv-contrib-dev
sudo ln -s /usr/local/include/opencv4/opencv2/ /usr/local/include/opencv2
sudo apt install ros-humble-image-transport ros-humble-image-transport-plugins

sudo apt-get install librealsense2-dev

