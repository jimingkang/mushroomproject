cmake_minimum_required(VERSION 3.8)

project(moveit2_motion_planning)

# Find necessary packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Create executable
add_executable(moveit2_motion_planning src/mushroom_planning.cpp)

# Link dependencies
ament_target_dependencies(moveit2_motion_planning 
  rclcpp 
  moveit_ros_planning_interface 
  moveit_core
  moveit_msgs 
  geometry_msgs 
  tf2_ros
)

# Install target
install(TARGETS moveit2_motion_planning
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

