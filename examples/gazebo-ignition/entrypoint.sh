#!/bin/bash
set -e

# Source ROS2 setup
source /opt/ros/jazzy/setup.bash

# Set display for GUI
export DISPLAY=${DISPLAY:-:0}

# Set environment variables for headless rendering if needed
export MESA_GL_VERSION_OVERRIDE=3.3
export LIBGL_ALWAYS_SOFTWARE=1

echo "=== Gazebo Modern ROS2 Jazzy Environment ==="
echo "Available commands:"
echo "  ros2 launch /workspace/launch/test_world.launch.py  - Launch SDF world only"
echo "  ros2 launch /workspace/launch/test_map.launch.py    - Launch with map server + RViz + TurtleBot3"
echo "  ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=/workspace/maps/map_ros2.yaml  - Start map server"
echo "  gz sim /workspace/maps/world_ignition.sdf          - Launch Gazebo Ignition directly"
echo ""
echo "Map files available in /workspace/maps/"
ls -la /workspace/maps/ 2>/dev/null || echo "  (No map files found - please export maps from the main app)"

# Execute the command passed to docker run
exec "$@"