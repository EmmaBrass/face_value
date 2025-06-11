#!/bin/bash
set -e

# Source ROS2 jazzy environment
source /opt/ros/jazzy/setup.sh

# Source workspace overlay (already built in image)
source /root/ros2_ws/install/setup.sh

# Start interactive shell or execute passed command
exec "$@"
