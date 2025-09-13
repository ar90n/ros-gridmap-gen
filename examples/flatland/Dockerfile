FROM ros:noetic-ros-base

# Install dependencies
RUN apt-get update && apt-get install -y \
    git build-essential \
    ros-noetic-tf2-ros \
    ros-noetic-tf2-geometry-msgs \
    ros-noetic-map-server \
    ros-noetic-rviz \
    python3-catkin-tools \
    qtbase5-dev \
    libopencv-dev \
    liblua5.1-0-dev \
    && rm -rf /var/lib/apt/lists/*

# Create catkin workspace and clone Flatland
RUN mkdir -p /catkin_ws/src
WORKDIR /catkin_ws/src
RUN git clone https://github.com/avidbots/flatland.git

# Install ROS dependencies
WORKDIR /catkin_ws
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y"

# Build the workspace
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && \
    catkin_make"

# Set up workspace
RUN mkdir -p /workspace
WORKDIR /workspace

# Copy launch files
COPY launch/ /workspace/launch/
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]