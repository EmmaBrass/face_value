FROM ros:jazzy

# Install build tools
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Copy ROS workspace into the container
COPY ./ros_ws /ros_ws
WORKDIR /ros_ws

# Build the workspace
RUN bash -c "source /opt/ros/jazzy/setup.bash && colcon build"

# Source and launch by default (optional)
CMD ["bash"]
