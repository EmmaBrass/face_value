FROM ros:jazzy-ros-core

# Install dependenciescd src
RUN apt update && apt install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pip \
 && pip3 install ollama \
 && rm -rf /var/lib/apt/lists/*

# Initialize rosdep (if not done in base image)
RUN rosdep init || true
RUN rosdep update

# Set working directory
WORKDIR /root/ros2_ws

# Copy your workspace source code into the image
COPY ros_pi_ws/ /root/ros2_ws/

# Install dependencies from rosdep (ignoring source packages)
RUN rosdep install --from-paths src --ignore-src -r -y || true

# Build your workspace
RUN . /opt/ros/jazzy/setup.sh && colcon build --symlink-install

# Copy helper script to container
COPY docker/setup_and_run.sh /root/setup_and_run.sh
RUN chmod +x /root/setup_and_run.sh

# Set entrypoint to your script
ENTRYPOINT ["/root/setup_and_run.sh"]
