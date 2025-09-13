FROM osrf/ros:noetic-desktop-full

# Install additional packages
RUN apt-get update && apt-get install -y \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-map-server \
    ros-noetic-turtlebot3-gazebo \
    ros-noetic-turtlebot3-navigation \
    && rm -rf /var/lib/apt/lists/*

# Set up workspace
RUN mkdir -p /workspace/src
WORKDIR /workspace

# Copy launch files
COPY launch/ /workspace/launch/
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Set environment variables for TurtleBot3
ENV TURTLEBOT3_MODEL=burger

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]