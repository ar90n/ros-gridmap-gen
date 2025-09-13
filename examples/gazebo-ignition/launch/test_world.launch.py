#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # World file path
    world_path = '/workspace/maps/world_ignition.sdf'
    
    # Check if world file exists and construct gz command
    if os.path.exists(world_path):
        gz_args = [world_path]
        print(f"Loading world file: {world_path}")
    else:
        gz_args = []
        print(f"Warning: World file not found at {world_path}")
        print("Please export maps from ROS GridMap Generator first")
        print("Starting empty Gazebo world...")
    
    # Launch Gazebo Ignition directly with gz sim command
    gazebo_launch = ExecuteProcess(
        cmd=['gz', 'sim'] + gz_args,
        output='screen'
    )
    
    return LaunchDescription([
        gazebo_launch,
    ])