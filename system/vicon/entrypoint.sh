#!/bin/bash

HAS_VEHICLE_CONFIG=false

# If we have a vehicle.config file, assume we are running on a real vehicle
if [ -f "/etc/starling/vehicle.config" ]; then 
    echo "Found vehicle.config"
    HAS_VEHICLE_CONFIG=true
    # Source VEHICLE_MAVLINK_SYSID, VEHICLE_NAME, VEHICLE_FCU_URL and VEHICLE_FIRMWARE
    source /etc/starling/vehicle.config
else
    exit 1;
fi

if [ ! -v $VEHICLE_NAMESPACE ]; then
    # If set Ensure VEHICLE_NAMESPACE is a valid topic name
    # Replace all '-' with '_'
    export VEHICLE_NAMESPACE=${VEHICLE_NAMESPACE//-/_}
    echo "VEHICLE_NAMESPACE setting to $VEHICLE_NAMESPACE"
else
    echo "VEHICLE_NAMESPACE not set, defaulting to vehicle_${VEHICLE_MAVLINK_SYSID}"
    export VEHICLE_NAMESPACE=vehicle_${VEHICLE_MAVLINK_SYSID}
fi

if [ -v $VEHICLE_NAME ]; then
    # If no vehicle name
    echo "VEHICLE_NAME was not set"
    exit 1;
else
    echo "VEHICLE_NAME set to $VEHICLE_NAME"
    export VEHICLE_NAME=$VEHICLE_NAME
fi

# Setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"

# Source the local workspace
source "/ros_ws/install/setup.bash"

exec "$@"
