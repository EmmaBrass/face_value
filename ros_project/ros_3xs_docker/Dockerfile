FROM ros:jazzy

# Install build tools
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Set up workspace
WORKDIR /ros_ws
COPY ./src ./src

# Build the workspace
RUN bash -c "source /opt/ros/jazzy/setup.bash && colcon build"

# Source and launch by default (optional)
CMD ["bash"]
