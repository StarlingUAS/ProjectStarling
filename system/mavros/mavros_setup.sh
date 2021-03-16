#!/bin/bash

HAS_DRONE_CONFIG=false

# If we have a drone.config file, assume we are running on a real drone
if [ -f "/etc/drone.config" ]; then 
    echo "Found drone.config"
    HAS_DRONE_CONFIG=true
    # Source MAVLINK_SYSID, VICON_NAME, FCU_URL and FIRMWARE
    source /etc/drone.config
fi

function get_instance {
    # Attempt to get ordinal
    ORDINAL="${HOSTNAME##*-}"

    if [ -z "$ORDINAL" ]; then
        # Parsing ordinal failed, assume non-kubernetes container and use default of 0
        echo "0"
        return 2
    fi

    if ! [ "$1" -ge 0 ] 2>/dev/null; then
        # Ordinal is not an integer, assume non-kubernetes container and use default of 0
        echo "0"
        return 3
    fi

    echo $((PX4_INSTANCE_BASE + ORDINAL))
    }

# If MAVROS_TGT_SYSTEM is auto, then set instance id to take MAVROS_TGT_SYSTEM_BASE + ORDINAL from StatefulSet hostname
# Hostname is of the form '<stateful set name>-<ordinal>'
if [ "$MAVROS_TGT_SYSTEM" = "auto" ]; then
    echo "MAVROS_TGT_SYSTEM set to auto"
    if [ $HAS_DRONE_CONFIG = true ]; then 
        echo "MAVROS_TGT_SYSTEM set to MAVLINK_SYSID ($MAVLINK_SYSID) from /etc/drone.config"
        MAVROS_TGT_SYSTEM=$MAVLINK_SYSID
    else
        INSTANCE=$(get_instance)
        RESULT=$?
        MAVROS_TGT_SYSTEM=$((INSTANCE + 1));
        if [ $RESULT -ne 0 ]; then
            echo "Could not parse ordinal, using default instance of 0. MAVROS_TGT_SYSTEM will be 1"
        else
            echo "MAVROS_TGT_SYSTEM set to $MAVROS_TGT_SYSTEM (from hostname: $HOSTNAME)"
        fi
    fi
elif (($MAVROS_TGT_SYSTEM >= 1 && $MAVROS_TGT_SYSTEM <= 255 )); then
    echo "MAVROS_TGT_SYSTEM setting as specified: $MAVROS_TGT_SYSTEM"
else
    echo "MAVROS_TGT_SYSTEM is invalid. Must either be set to 'auto' or a integer between 1 and 255" 
    exit 1
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
    # If not set, then generate automatically
    if ! [ $HAS_DRONE_CONFIG = true ]; then
        # No drone.config, generate based on subparameters and the target system id dynamically
        INSTANCE=$(get_instance)
        MAVROS_FCU_URL="$MAVROS_FCU_CONN://$MAVROS_FCU_IP:$((MAVROS_FCU_UDP_BASE + INSTANCE))@"
        export MAVROS_FCU_URL=$MAVROS_FCU_URL
    fi
    echo "MAVROS_FCU_URL automatically set to: $MAVROS_FCU_URL"
else
    echo "MAVROS_FCU_URL setting to $MAVROS_FCU_URL"
fi
