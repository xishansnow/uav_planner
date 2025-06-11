#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('ros_gz_uav_application')
    
    # Declare launch arguments
    planner_type_arg = DeclareLaunchArgument(
        'planner_type',
        default_value='astar',
        description='Type of planner to use (astar, thetastar, arastar)'
    )
    
    octomap_file_arg = DeclareLaunchArgument(
        'octomap_file',
        default_value='',
        description='Path to octomap file (.bt)'
    )
    
    environment_file_arg = DeclareLaunchArgument(
        'environment_file',
        default_value='',
        description='Path to environment file (.yaml)'
    )
    
    max_planning_time_arg = DeclareLaunchArgument(
        'max_planning_time',
        default_value='5.0',
        description='Maximum planning time in seconds'
    )
    
    publish_path_arg = DeclareLaunchArgument(
        'publish_path',
        default_value='true',
        description='Whether to publish path'
    )
    
    publish_markers_arg = DeclareLaunchArgument(
        'publish_markers',
        default_value='true',
        description='Whether to publish visualization markers'
    )
    
    # Create global path planner node
    global_planner_node = Node(
        package='ros_gz_uav_application',
        executable='global_path_planner_node',
        name='global_path_planner',
        output='screen',
        parameters=[{
            'planner_type': LaunchConfiguration('planner_type'),
            'octomap_file': LaunchConfiguration('octomap_file'),
            'environment_file': LaunchConfiguration('environment_file'),
            'max_planning_time': LaunchConfiguration('max_planning_time'),
            'publish_path': LaunchConfiguration('publish_path'),
            'publish_markers': LaunchConfiguration('publish_markers'),
        }]
    )
    
    # Create RViz node for visualization
    rviz_config_file = os.path.join(pkg_dir, 'config', 'global_planner.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )
    
    return LaunchDescription([
        planner_type_arg,
        octomap_file_arg,
        environment_file_arg,
        max_planning_time_arg,
        publish_path_arg,
        publish_markers_arg,
        global_planner_node,
        rviz_node
    ]) 