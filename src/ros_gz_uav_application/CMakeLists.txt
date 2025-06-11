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
find_package(PkgConfig REQUIRED)
pkg_check_modules(OCTOMAP REQUIRED octomap)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

# Include directories
include_directories(include)
include_directories(${OCTOMAP_INCLUDE_DIRS})
include_directories(${SBPL_INCLUDE_DIRS})
include_directories(${YAML_CPP_INCLUDE_DIRS})

# Create the SBPL path planner executable
add_executable(sbpl_path_planner_node 
  src/sbpl_path_planner_node.cpp
  src/sbpl_planner.cpp
  src/environment_voxel3d.cpp
)

# Create the global path planner executable
add_executable(global_path_planner_node
  src/global_path_planner_node.cpp
  src/global_planner_base.cpp
  src/astar_planner.cpp
  src/thetastar_planner.cpp
  src/arastar_planner.cpp
  src/planner_factory.cpp
  src/environment_voxel3d.cpp
)

# Link libraries for SBPL planner
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
)

# Link libraries for global planner
ament_target_dependencies(global_path_planner_node
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
)

# Link SBPL and octomap libraries for SBPL planner
target_link_libraries(sbpl_path_planner_node
  ${SBPL_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

# Link octomap libraries for global planner
target_link_libraries(global_path_planner_node
  ${OCTOMAP_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

# Install the executables
install(TARGETS
  sbpl_path_planner_node
  global_path_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
