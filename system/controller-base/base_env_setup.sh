#!/bin/bash
echo "---- controller base setup START ------------"

HAS_VEHICLE_CONFIG=false

# If we have a vehicle.config file, assume we are running on a real vehicle
if [ -f "/etc/starling/vehicle.config" ]; then
    echo "Found vehicle.config"
    HAS_VEHICLE_CONFIG=true
    # Source VEHICLE_MAVLINK_SYSID, VEHICLE_NAME, VEHICLE_FCU_URL and VEHICLE_FIRMWARE
    source /etc/starling/vehicle.config
fi


if [ ! -v $VEHICLE_NAMESPACE ]; then
    # If set Ensure VEHICLE_NAMESPACE is a valid topic name
    # Replace all '-' with '_'
    export VEHICLE_NAMESPACE=${VEHICLE_NAMESPACE//-/_}
    echo "VEHICLE_NAMESPACE setting to $VEHICLE_NAMESPACE"
else
    echo "VEHICLE_NAMESPACE not set, default to launchfile defaults"
fi

# Check if using a discovery server or not
if [[ -v $ROS_DISCOVERY_SERVER ]] && [[ ! -z "$ROS_DISCOVERY_SERVER" ]]; then
    echo "ROS DISCOVERY SERVER SET TO: $ROS_DISCOVERY_SERVER"
    export ROS_DISCOVERY_SERVER
else
    echo "ROS_DISCOVERY_SERVER Not Set"
fi

echo "---- controller base setup END ------------"