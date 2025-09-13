#!/bin/bash
set -e

# Source ROS setup
source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash

# Set display for GUI
export DISPLAY=${DISPLAY:-:0}

echo "=== Flatland ROS Noetic Environment ==="
echo "Available commands:"
echo "  roslaunch flatland_server server.launch world_path:=/workspace/maps/world_flatland.yaml  - Launch Flatland with generated world"
echo "  roslaunch /workspace/launch/test_map.launch     - Launch Flatland with map"
echo "  roslaunch /workspace/launch/test_simple.launch  - Launch minimal Flatland setup"
echo "  rosrun map_server map_server /workspace/maps/map_ros1.yaml  - Start map server only"
echo ""
echo "Map files available in /workspace/maps/"
ls -la /workspace/maps/*.yaml /workspace/maps/*.pgm 2>/dev/null || echo "  (No map files found - please export maps from the main app)"

# Execute the command passed to docker run
exec "$@"