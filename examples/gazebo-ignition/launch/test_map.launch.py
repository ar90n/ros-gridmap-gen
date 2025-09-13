#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # World file path  
    world_path = '/workspace/maps/world_ignition.sdf'
    map_file = '/workspace/maps/map_ros2.yaml'
    
    # Map server
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': map_file,
            'use_sim_time': True,
        }]
    )
    
    # Lifecycle manager for map server
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map_server',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['map_server']
        }]
    )
    
    # Static transform
    static_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )
    
    # Launch Gazebo with custom world file
    if os.path.exists(world_path):
        gazebo_launch = ExecuteProcess(
            cmd=['gz', 'sim', world_path],
            output='screen'
        )
    else:
        gazebo_launch = ExecuteProcess(
            cmd=['gz', 'sim'],
            output='screen'
        )
    
    # Use nav2_minimal_tb3_sim to spawn TurtleBot3
    tb3_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_minimal_tb3_sim'),
                'launch',
                'spawn_tb3.launch.py'
            ])
        ])
    )
    
    # RViz with default config
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )
    
    return LaunchDescription([
        map_server,
        lifecycle_manager, 
        static_transform,
        gazebo_launch,
        tb3_spawn,
        rviz,
    ])