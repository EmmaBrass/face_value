FROM ros:jazzy

# Set non-interactive mode for apt
ENV DEBIAN_FRONTEND=noninteractive

# Install build tools
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    git \
    && rm -rf /var/lib/apt/lists/*

# Copy ROS workspace into the container
COPY ./ros_ws /ros_ws
WORKDIR /ros_ws

# Build the workspace
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && colcon build"

# Source setup and run launch file
CMD ["/bin/bash", "-c", "source /opt/ros/jazzy/setup.bash && source /ros_ws/install/setup.bash && ros2 launch rol_pi cam_launch.py"]

