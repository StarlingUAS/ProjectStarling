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

# If PX4_INSTANCE is zero, then set instance id to IP address last digit
if [ "$PX4_INSTANCE" -eq "0" ]; then
    IPADDR=`ifconfig eth0  | grep 'inet' | awk '{print $2}'`
    read A B C D <<< "${IPADDR//./ }"
    export PX4_INSTANCE=$((D-1));
    echo "PX4_INSTANCE set to $PX4_INSTANCE from IP ADDRESS: $IPADDR (was zero)"
else
    echo "PX4_INSTANCE setting as specified: $PX4_INSTANCE"
fi

/ros_entrypoint.sh "$@"