#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash"

# Setup additional environment (e.g. gazebo paths)
source /ros.env

exec "$@"