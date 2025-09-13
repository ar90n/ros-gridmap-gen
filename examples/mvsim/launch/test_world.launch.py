#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Check if generated world file exists
    default_world = '/workspace/maps/world_mvsim.xml'
    
    if not os.path.exists(default_world):
        print(f"Warning: Generated world file not found at {default_world}")
        print("Please export maps from ROS GridMap Generator first")
        print("Using MVSim's built-in demo world")
        default_world = ''  # Let MVSim use its default
    
    # Declare launch arguments
    world_file_arg = DeclareLaunchArgument(
        'world_file',
        default_value=default_world,
        description='Path to the world file'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to launch RViz'
    )
    
    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run in headless mode'
    )
    
    # Include the main MVSim launch file
    mvsim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('mvsim'),
                'launch',
                'launch_world.launch.py'
            ])
        ]),
        launch_arguments={
            'world_file': LaunchConfiguration('world_file'),
            'use_rviz': LaunchConfiguration('use_rviz'),
            'headless': LaunchConfiguration('headless')
        }.items()
    )
    
    return LaunchDescription([
        world_file_arg,
        use_rviz_arg,
        headless_arg,
        mvsim_launch
    ])