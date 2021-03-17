#!/bin/bash

if [ ! -v $MAVLINK_SYSID ]; then
    MAVROS_TGT_SYSTEM = $MAVLINK_SYSID
fi

# If MAVROS_TGT_SYSTEM is zero, then set instance id to take MAVROS_TGT_SYSTEM_BASE + ORDINAL from StatefulSet hostname
# Hostname is of the form '<stateful set name>-<ordinal>'
if [ "$MAVROS_TGT_SYSTEM" == "auto" ]; then
    echo "MAVROS_TGT_SYSTEM set to auto"
    if [ -f "/etc/drone.config" ]; then 
        echo "MAVROS_TGT_SYSTEM will be read from file not yet implemented, set to 1 for now"
        MAVROS_TGT_SYSTEM=1 # TODO
    else
        ORDINAL="${HOSTNAME##*-}"
        if [ ! -z "$ORDINAL" ]; then
            MAVROS_TGT_SYSTEM=$((MAVROS_TGT_SYSTEM_BASE + ORDINAL + 1));
            echo "MAVROS_TGT_SYSTEM set to $MAVROS_TGT_SYSTEM (from hostname: $HOSTNAME), was set to zero and /etc/drone.config not found"
        else
            MAVROS_TGT_SYSTEM=1;
            echo "MAVROS_TGT_SYSTEM set to $MAVROS_TGT_SYSTEM, ordinal could not parsed from hostname: $HOSTNAME and /etc/drone.config not found"
        fi
    fi
elif (($MAVROS_TGT_SYSTEM >= 1 && $MAVROS_TGT_SYSTEM <= 256 )); then
    echo "MAVROS_TGT_SYSTEM setting as specified: $MAVROS_TGT_SYSTEM"
else
    echo "MAVROS_TGT_SYSTEM (set to $MAVROS_TGT_SYSTEM) is invalid, setting to 1. Must either be set to 'auto' where it will either look  or number between 1 and 256" 
    MAVROS_TGT_SYSTEM=1
fi

export MAVROS_TGT_SYSTEM=$MAVROS_TGT_SYSTEM


if [ ! -v $VEHICLE_NAMESPACE ]; then
    # If set Ensure VEHICLE_NAMESPACE is a valid topic name
    # Replace all '-' with '_'
    export VEHICLE_NAMESPACE=${VEHICLE_NAMESPACE//-/_}
    echo "VEHICLE_NAMESPACE setting to $VEHICLE_NAMESPACE"
else
    echo "VEHICLE_NAMESPACE not set, default to mavros_bridge.launch.xml default"
fi


if [ -v $MAVROS_FCU_URL ]; then
    # If not set, then autogenerate based on subparameters and the target system id dynamically
    export MAVROS_FCU_URL="$MAVROS_FCU_CONN://$MAVROS_FCU_IP:$((MAVROS_FCU_UDP_BASE + MAVROS_TGT_SYSTEM - 1))@"
    echo "MAVROS_FCU_URL not set, defaulting to autogenerated: $MAVROS_FCU_URL"
else
    echo "MAVROS_FCU_URL setting to $MAVROS_FCU_URL"
fi