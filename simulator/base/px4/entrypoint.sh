#!/bin/bash

# If PX4_SIM_HOST is set, lookup IP for simulator
if [ -n "$PX4_SIM_HOST" ]; then
    echo "Looking up sim host"
    export PX4_SIM_IP=$(getent ahostsv4 $PX4_SIM_HOST | head -1 | awk '{print $1}')
else
    echo "PX4_SIM_HOST not set, assuming loopback"
fi

# If PX4_OFFBOARD_HOST is set, lookup IP for offboard
if [ -n "$PX4_OFFBOARD_HOST" ]; then
    echo "Looking up offboard host"
    export PX4_OFFBOARD_IP=$(getent ahostsv4 $PX4_OFFBOARD_HOST | head -1 | awk '{print $1}')
else
    echo "PX4_OFFBOARD_HOST not set, assuming loopback"
fi

/ros_entrypoint.sh "$@"