cmake_minimum_required(VERSION 3.5)

project(ros_gz_uav_application)

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(angles REQUIRED)
find_package(sbpl REQUIRED)

# Include directories
include_directories(include)

# Create the SBPL path planner executable
add_executable(sbpl_path_planner_node 
  src/sbpl_path_planner_node.cpp
  src/sbpl_planner.cpp
)

# Link libraries
ament_target_dependencies(sbpl_path_planner_node
  rclcpp
  rclcpp_action
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav2_msgs
  nav2_util
  nav2_costmap_2d
  nav2_core
  pluginlib
  angles
  sbpl
)

# Install the executable
install(TARGETS
  sbpl_path_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
