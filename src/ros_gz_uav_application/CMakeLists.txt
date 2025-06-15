cmake_minimum_required(VERSION 3.5)

project(ros_gz_uav_application)

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(angles REQUIRED)
find_package(std_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)
# find_package(PkgConfig REQUIRED)
# pkg_check_modules(OCTOMAP REQUIRED octomap)
# pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
# pkg_check_modules(OCTOMAP_MSGS REQUIRED octomap_msgs)
# 注意： pkg_check_modules 和 find_package 的区别，
# 前者适用于 pkg-config 的库（依赖于 .pc 文件）， 后者适用于 cmake 的库（Config.cmake 文件 或 Find<Package>.cmake 文件）

# Find OpenMP for parallel processing
find_package(OpenMP)
if(OpenMP_CXX_FOUND)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    message(STATUS "OpenMP found and enabled for parallel processing")
else()
    message(WARNING "OpenMP not found. Parallel processing will be disabled.")
endif()

# Add voxelization subdirectory
# add_subdirectory(src/voxelization)

# Include directories
include_directories(include)
include_directories(${OCTOMAP_INCLUDE_DIRS})
include_directories(${YAML_CPP_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})


# Create the global path planner executable
add_executable(global_path_planner_node
  src/nodes/global_path_planner_node.cpp
  src/single_global/global_planner_base.cpp
  src/single_global/astar_planner.cpp
  src/single_global/thetastar_planner.cpp
  src/single_global/arastar_planner.cpp
  src/single_global/jps_planner.cpp
  src/single_global/multiscale_astar_planner.cpp
  src/single_global/planner_factory.cpp
  src/env/environment_voxel3d.cpp
  src/smoother/smoother_factory.cpp
  src/smoother/smoother_normal.cpp
  src/smoother/smoother_bezier.cpp
  src/smoother/smoother_bspline.cpp
  src/smoother/smoother_minvo.cpp
)

# Create the test executable
add_executable(test_3d_planning
  src/nodes/test_3d_planning.cpp
  src/single_global/global_planner_base.cpp
  src/single_global/astar_planner.cpp
  src/single_global/thetastar_planner.cpp
  src/single_global/arastar_planner.cpp
  src/single_global/jps_planner.cpp
  src/single_global/multiscale_astar_planner.cpp
  src/single_global/planner_factory.cpp
  src/env/environment_voxel3d.cpp
  src/smoother/smoother_factory.cpp
  src/smoother/smoother_normal.cpp
  src/smoother/smoother_bezier.cpp
  src/smoother/smoother_bspline.cpp
  src/smoother/smoother_minvo.cpp
)

# Create the simple visualization test executable
add_executable(simple_visualization_test
  test/simple_visualization_test.cpp
)

# Create the normal smoother test executable
add_executable(test_normal_smoother
  test/test_normal_smoother.cpp
  src/smoother/smoother_normal.cpp
)

# Create voxelization test executable
# add_executable(test_voxelization
#   src/voxelization/test/test_voxelization.cpp
# )

# Create migration example executable
# add_executable(migrate_obstacles
#   src/voxelization/examples/migrate_obstacles.cpp
# )

# Add include directories for test executable
target_include_directories(test_3d_planning PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/include/single_global
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_include_directories(global_path_planner_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/include/single_global
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# Create the octomap publisher executable
add_executable(octomap_publisher_node
  src/nodes/octomap_publisher_node.cpp
  src/env/environment_voxel3d.cpp
)

# Link libraries for global path planner
ament_target_dependencies(global_path_planner_node
  rclcpp
  rclcpp_action
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav2_msgs
  nav2_util
  nav2_costmap_2d
  nav2_core
  pluginlib
  angles      
  std_msgs    
  octomap_msgs
  yaml-cpp
)

target_link_libraries(global_path_planner_node
  octomap
  octomath
  yaml-cpp
  # voxelization  
)

# Link libraries for test executable
target_link_libraries(test_3d_planning
  octomap
  octomath
  yaml-cpp
  # voxelization
)

ament_target_dependencies(test_3d_planning
  rclcpp
  visualization_msgs
  geometry_msgs
  std_msgs
  tf2_geometry_msgs
  tf2
  tf2_ros
  octomap_msgs
  yaml-cpp
)

# Link libraries for voxelization test
# target_link_libraries(test_voxelization
#   voxelization
# )

# Link libraries for migration example
# target_link_libraries(migrate_obstacles
#   voxelization
# )

# Link libraries for simple visualization test
ament_target_dependencies(simple_visualization_test
  rclcpp
  visualization_msgs
  geometry_msgs
  std_msgs
)

# Link libraries for octomap publisher
ament_target_dependencies(octomap_publisher_node
  rclcpp
  octomap_msgs
  tf2_ros
  geometry_msgs
  std_msgs
)

# Link octomap libraries for octomap publisher
target_link_libraries(octomap_publisher_node
  ${OCTOMAP_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

# Install the executables
install(TARGETS
  global_path_planner_node
  octomap_publisher_node
  test_3d_planning
  simple_visualization_test
  test_normal_smoother
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install voxelization headers
# install(DIRECTORY
#   src/voxelization/include/
#   DESTINATION include/${PROJECT_NAME}/voxelization
# )

ament_package()

# Link libraries for normal smoother test
ament_target_dependencies(test_normal_smoother
  rclcpp
  geometry_msgs
)
