#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash"

if [ -d "/home/ctrl_user/dev_ws/install" ]; then
    source "/home/ctrl_user/dev_ws/install/setup.bash"
fi

exec "$@"
