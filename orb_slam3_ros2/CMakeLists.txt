cmake_minimum_required(VERSION 3.5)
project(orbslam3)

# You should set the PYTHONPATH to your own python site-packages path
set(ENV{PYTHONPATH} "/opt/ros/humble/lib/python3.8/site-packages/")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
#jimmy add 
find_package(OpenCV 4 QUIET)
#set(cv_bridge_DIR /usr/local/share/cv_bridge/cmake)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge_orbslam3 REQUIRED)
find_package(message_filters REQUIRED)
find_package(Sophus REQUIRED)
find_package(Pangolin REQUIRED)
find_package(ORB_SLAM3 REQUIRED)

# adding for point cloud viewer and mapper
find_package(Eigen3 3.1.0 REQUIRED)
find_package(PCL 1.10 REQUIRED )
find_package(VTK 7.1 REQUIRED )


find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
include_directories(
  include
  ${ORB_SLAM3_ROOT_DIR}/include
  ${ORB_SLAM3_ROOT_DIR}/include/CameraModels
  /opt/ros/humble/include/image_transport
  )
link_directories(
  include
)

#add_executable(mono
#  src/monocular/mono.cpp
#  src/monocular/monocular-slam-node.cpp
#)
#ament_target_dependencies(mono rclcpp sensor_msgs cv_bridge_orbslam3 ORB_SLAM3 Pangolin OpenCV)

add_executable(rgbd
  src/rgbd/rgbd.cpp
  src/rgbd/rgbd-slam-node.cpp
)
ament_target_dependencies(rgbd nav_msgs  pcl_conversions rclcpp sensor_msgs geometry_msgs cv_bridge_orbslam3 message_filters ORB_SLAM3 Pangolin OpenCV)

#add_executable(stereo
#  src/stereo/stereo.cpp
#  src/stereo/stereo-slam-node.cpp
#)
#ament_target_dependencies(stereo rclcpp sensor_msgs cv_bridge_orbslam3 message_filters
#ORB_SLAM3 Pangolin)

#add_executable(stereo-inertial
#  src/stereo-inertial/stereo-inertial.cpp
#  src/stereo-inertial/stereo-inertial-node.cpp
#)
#ament_target_dependencies(stereo-inertial rclcpp sensor_msgs cv_bridge_orbslam3 ORB_SLAM3 Pangolin OpenCV)

install(TARGETS  rgbd 
	#stereo 
	#	stereo-inertial
  DESTINATION lib/${PROJECT_NAME})

# Install launch files.
#install(DIRECTORY launch config vocabulary
#  DESTINATION share/${PROJECT_NAME}/)

ament_package()

