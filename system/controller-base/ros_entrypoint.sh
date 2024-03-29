#!/bin/bash
set -e

# setup ros1 environment
source "/opt/ros/$ROS_DISTRO/setup.bash"

# Source the local workspace
source "/ros_ws/install/setup.bash"

# Setup additional environment (e.g. gazebo paths)
source /ros.env

# Attempt to source additional env files (possibly volume mounted)
for folder in $(find /ros.env.d -mindepth 1 -maxdepth 1 -type d -name "[!.]*"); do
    if [ -f ${folder}/setup.bash ]; then
        source ${folder}/setup.bash
    fi
done

exec "$@"
