FROM ros:jazzy

# Set non-interactive mode for apt
ENV DEBIAN_FRONTEND=noninteractive

# Install build tools and camera dependencies
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    v4l-utils \
    && rm -rf /var/lib/apt/lists/*

# Copy ROS workspace into the container
COPY ./ros_ws /ros_ws
WORKDIR /ros_ws

# Build the workspace
RUN bash -c "source /opt/ros/jazzy/setup.bash && colcon build"

# Source and launch by default (optional)
CMD ["bash"]
