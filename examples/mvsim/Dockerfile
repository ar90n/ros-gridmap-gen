FROM osrf/ros:humble-desktop

# Install additional packages
RUN apt-get update && apt-get install -y \
    ros-humble-mvsim \
    ros-humble-nav2-map-server \
    ros-humble-rviz2 \
    && rm -rf /var/lib/apt/lists/*

# Set up workspace
RUN mkdir -p /workspace/src
WORKDIR /workspace

# Copy entrypoint
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]