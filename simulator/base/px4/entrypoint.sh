#!/bin/bash

# If PX4_SIM_HOST is set, lookup IP for simulator
if [ -n "$PX4_SIM_HOST" ]; then
    echo "Looking up sim host"
    export PX4_SIM_IP=$(getent ahostsv4 $PX4_SIM_HOST | head -1 | awk '{print $1}')
else
    echo "PX4_SIM_HOST not set, assuming loopback"
fi
echo "Setting PX4_SIM_IP to $PX4_SIM_IP"

# If PX4_OFFBOARD_HOST is set, lookup IP for offboard
if [ -n "$PX4_OFFBOARD_HOST" ]; then
    echo "Looking up offboard host"
    export PX4_OFFBOARD_IP=$(getent ahostsv4 $PX4_OFFBOARD_HOST | head -1 | awk '{print $1}')
else
    echo "PX4_OFFBOARD_HOST not set, assuming loopback"
fi
echo "Setting PX4_OFFBOARD_IP to $PX4_OFFBOARD_IP"

# If PX4_INSTANCE is "ordinal", then set instance id to take PX4_INSTANCE_BASE + ORDINAL from StatefulSet hostname
# Therefore SYSID is set to PX4_INSTANCE_BASE + ORDINAL + 1
# Hostname is of the form '<stateful set name>-<ordinal>'
if [ "$PX4_INSTANCE" == "ordinal" ]; then
    ORDINAL="${HOSTNAME##*-}"
    export PX4_INSTANCE=$((PX4_INSTANCE_BASE + ORDINAL));
    echo "PX4_INSTANCE was 'ordinal' therefore set to $PX4_INSTANCE (from base: $PX4_INSTANCE_BASE and hostname: $HOSTNAME)"
elif (($PX4_INSTANCE >= 0 && $PX4_INSTANCE <= 254 )); then
    echo "PX4_INSTANCE setting as specified: $PX4_INSTANCE"
else
    echo "PX4_INSTANCE (set to $PX4_INSTANCE) is invalid, setting to 0. Must either be set to 'ordinal' or number between 0 and 254"
    export PX4_INSTANCE=0;
fi
export PX4_SYSID=$((PX4_INSTANCE + 1))
echo "PX4 SYSID is set to $PX4_SYSID"

/ros_entrypoint.sh "$@"
