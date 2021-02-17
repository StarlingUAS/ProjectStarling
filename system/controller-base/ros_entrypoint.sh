#!/bin/bash
set -e

# setup ros1 environment
source "/opt/ros/$ROS_DISTRO/setup.bash"

# Source the local workspace
source "/ros_ws/install/setup.bash"

exec "$@"
