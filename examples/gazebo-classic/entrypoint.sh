#!/bin/bash
set -e

# Source ROS setup
source /opt/ros/noetic/setup.bash

# Set display for GUI
export DISPLAY=${DISPLAY:-:0}

# Set Gazebo model path
export GAZEBO_MODEL_PATH=/opt/ros/noetic/share/turtlebot3_gazebo/models:$GAZEBO_MODEL_PATH

echo "=== Gazebo Classic ROS Noetic Environment ==="
echo "Available commands:"
echo "  roslaunch /workspace/launch/test_world.launch  - Launch SDF world"
echo "  roslaunch /workspace/launch/test_map.launch    - Launch with map server"
echo "  rosrun map_server map_server /workspace/maps/map_ros1.yaml  - Start map server"
echo ""
echo "Map files available in /workspace/maps/"
ls -la /workspace/maps/ 2>/dev/null || echo "  (No map files found - please export maps from the main app)"

# Execute the command passed to docker run
exec "$@"