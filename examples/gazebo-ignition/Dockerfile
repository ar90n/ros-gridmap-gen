FROM osrf/ros:jazzy-desktop

# Install Gazebo Modern (included in Jazzy) and Navigation packages
RUN apt-get update && apt-get install -y \
    ros-jazzy-ros-gz \
    ros-jazzy-nav2-bringup \
    ros-jazzy-nav2-minimal-tb3-sim \
    ros-jazzy-rviz2 \
    python3-lxml \
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
ENV GAZEBO_MODEL_PATH=/opt/ros/humble/share/turtlebot3_gazebo/models

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]