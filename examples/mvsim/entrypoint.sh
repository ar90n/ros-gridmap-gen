#!/bin/bash
set -e

# Source ROS2 setup
source /opt/ros/humble/setup.bash

# Set display for GUI
export DISPLAY=${DISPLAY:-:0}

echo "=== MVSim ROS2 Humble Environment ==="
echo "Available commands:"
echo ""
echo "Launch with generated world + robot (recommended):"
echo "  ros2 launch ./launch/test_world.launch.py                           - Uses generated world with robot"
echo ""
echo "Launch with specific world:"
echo "  ros2 launch ./launch/test_world.launch.py world_file:=/workspace/maps/world_mvsim.xml"
echo ""
echo "Launch options:"
echo "  ros2 launch ./launch/test_world.launch.py use_rviz:=false           - Without RViz"
echo "  ros2 launch ./launch/test_world.launch.py headless:=true            - Headless mode"
echo ""
echo "Official MVSim demos:"
echo "  ros2 launch mvsim mvsim_tutorial/demo_warehouse.launch.py           - Official warehouse demo"
echo ""
echo "Map Server (separate terminal):"
echo "  ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=/workspace/maps/map_ros2.yaml"
echo ""
echo "World files available:"
ls -la /workspace/maps/*.xml 2>/dev/null || echo "  (No world files found - please export maps from the main app)"
echo ""
echo "Map files available:"
ls -la /workspace/maps/*.yaml /workspace/maps/*.pgm 2>/dev/null || echo "  (No map files found - please export maps from the main app)"

# Execute the command passed to docker run
exec "$@"