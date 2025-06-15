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
        default_value=os.path.join(pkg_dir, 'config', 'planning_test_separated.rviz'),
        description='Path to RViz configuration file'
    )
    
    # Create the 3D planning test node
    planning_test_node = Node(
        package='ros_gz_uav_application',
        executable='test_3d_planning',
        name='planning_test_node',
        output='screen',
        parameters=[{
            'use_sim_time': False
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
        planning_test_node,
        rviz_node
    ]) 