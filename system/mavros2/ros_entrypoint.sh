#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash"

# setup the local environment
source "/ros_ws/install/setup.bash"

# Source the mavros_setup for any user defined edits to the environment
source "/ros_ws/mavros_setup.sh"

exec "$@"
