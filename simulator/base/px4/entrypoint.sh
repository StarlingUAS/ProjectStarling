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
if [ "$PX4_INSTANCE" -eq "ordinal" ]; then
    ORDINAL="${HOSTNAME##*-}"
    export PX4_INSTANCE=$((PX4_SYSID_SITL_BASE + ORDINAL-1));
    echo "PX4_INSTANCE is zero therefore set to $PX4_INSTANCE (from hostname: $HOSTNAME)"
elif (($PX4_INSTANCE >= 0 && $PX4_INSTANCE <= 255 )); then
    echo "PX4_INSTANCE setting as specified: $PX4_INSTANCE"
else
    echo "PX4_INSTANCE is invalid. Must either be set to 'ordinal' or number between 0 and 255"
fi
export PX4_SYSID=$((PX4_INSTANCE + 1))
echo "PX4 SYSID is set to $PX4_SYSID"

/ros_entrypoint.sh "$@"