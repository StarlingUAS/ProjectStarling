#!/bin/bash
if [[ -f "/etc/starling/vehicle.config" ]]; then
    echo "Sourcing local vehicle configuration"
    source "/etc/starling/vehicle.config"
fi

if [[ -f "/ros_ws/install/setup.bash" ]]; then
    echo "Sourcing local install setup.bash"
    source "/ros_ws/install/setup.bash"
fi

if [ ! -v $VEHICLE_MAVLINK_SYSID ]; then
    export VEHICLE_MAVLINK_SYSID=$VEHICLE_MAVLINK_SYSID
    echo "VEHICLE_MAVLINK_SYSID setting to $VEHICLE_MAVLINK_SYSID"
else
    export VEHICLE_MAVLINK_SYSID=1
    echo "VEHICLE_MAVLINK_SYSID not set, default to 1"
fi

ZENOH_ALLOW_TOPICS="/mavros/|$ZENOH_ALLOW_TOPICS"

export RUST_LOG=warn

if [ ! -v $SERVER ]; then
    echo "Running Zenoh on Server"
    exec zenoh-bridge-dds \
        -d 20 \
        # --no-multicast-scouting \
        -l tcp/0.0.0.0:7447 
        # -e tcp/10.0.0.101:7447 \
        # -e tcp/10.0.0.102:7447 \
        # -e tcp/10.0.0.103:7447 \
        # -e tcp/10.0.0.104:7447 \
        # -e tcp/10.0.0.105:7447 \
else
    echo "Running Zenoh on Board Vehicle $VEHICLE_MAVLINK_SYSID, with ROS_DOMAIN_ID set to $VEHICLE_MAVLINK_SYSID"
    exec zenoh-bridge-dds \
        -d "$VEHICLE_MAVLINK_SYSID" \
        --no-multicast-scouting \
        -l tcp/0.0.0.0:7447 \
        --allow "$ZENOH_ALLOW_TOPICS"
fi
