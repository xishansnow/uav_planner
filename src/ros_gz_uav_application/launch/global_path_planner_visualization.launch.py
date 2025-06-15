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
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(pkg_dir, 'config', 'global_path_planner.rviz'),
        description='Path to RViz configuration file'
    )
    
    # Create the Global Path Planner node
    global_path_planner_node = Node(
        package='ros_gz_uav_application',
        executable='global_path_planner_node',
        name='global_path_planner_node',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'planner_type': 'astar',
            'planner_frequency': 1.0,
            'max_planning_time': 5.0,
            'goal_tolerance': 0.5,
            'robot_radius': 0.3,
            'map_resolution': 0.1,
            'origin_x': -100.0,
            'origin_y': -100.0,
            'origin_z': 0.0,
            'env_width': 200.0,
            'env_height': 200.0,
            'env_depth': 100.0,
            'voxel_width': 200,
            'voxel_height': 200,
            'voxel_depth': 100
        }]
    )
    
    # Create RViz node for visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        output='screen'
    )
    
    return LaunchDescription([
        rviz_config_arg,
        global_path_planner_node,
        rviz_node
    ]) 