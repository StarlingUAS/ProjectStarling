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

# If PX4_INSTANCE is zero, then set instance id to take PX4_SYSID_SITL_BASE + ORDINAL-1 from StatefulSet hostname
# Therefore SYSID is set to PX4_SYSID_SITL_BASE + ORDINAL
# Hostname is of the form '<stateful set name>-<ordinal>'
if [ "$PX4_INSTANCE" -eq "0" ]; then
    ORDINAL="${HOSTNAME##*-}"
    export PX4_INSTANCE=$((PX4_SYSID_SITL_BASE + ORDINAL-1));
    echo "PX4_INSTANCE is zero therefore set to $PX4_INSTANCE (from hostname: $HOSTNAME)"
else
    echo "PX4_INSTANCE setting as specified: $PX4_INSTANCE"
fi
echo "PX4 SYSID is set to $((PX4_INSTANCE + 1))"

/ros_entrypoint.sh "$@"