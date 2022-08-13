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
if [ "$DISCOVERY_SERVER" = "local" ]; then
    export ROS_DISCOVERY_SERVER="127.0.0.1:11811"
    echo "DISCOVERY_SERVER set to $DISCOVERY_SERVER, assigning ROS_DISCOVERY_SERVER to $ROS_DISCOVERY_SERVER"
elif [ "$DISCOVERY_SERVER" = "global" ]; then
    export ROS_DISCOVERY_SERVER="127.0.0.1:11811;$CENTRAL_ROS_DISCOVERY_SERVER"
    echo "DISCOVERY_SERVER set to $DISCOVERY_SERVER, assigning ROS_DISCOVERY_SERVER to $ROS_DISCOVERY_SERVER"
else
    echo "DISCOVERY_SERVER not set to local or global (set to $DISCOVERY_SERVER), ros2 uses default discovery mechanism"
fi

echo "---- controller base setup END ------------"