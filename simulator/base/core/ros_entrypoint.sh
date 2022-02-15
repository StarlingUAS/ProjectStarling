#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash"

# setup display environment
source /setup_display.sh

# Setup additional environment (e.g. gazebo paths)
source /ros.env

# Attempt to source additional env files (possibly volume mounted)
ROSENVD_LIST=$(find /ros.env.d -mindepth 1 -maxdepth 1 -type d -name "[!.]*")
for folder in $(echo $ROSENVD_LIST | xargs -n1 | sort | xargs ); do
    if [ -f ${folder}/setup.bash ]; then
        echo "---- ros_entrypoint.sh: sourcing ${folder}/setup.bash -----"
        source ${folder}/setup.bash
    fi
done

exec "$@"