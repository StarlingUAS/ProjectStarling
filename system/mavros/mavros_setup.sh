#!/bin/bash

HAS_VEHICLE_CONFIG=false

# If we have a vehicle.config file, assume we are running on a real vehicle
if [ -f "/etc/starling/vehicle.config" ]; then 
    echo "Found vehicle.config"
    HAS_VEHICLE_CONFIG=true
    # Source VEHICLE_MAVLINK_SYSID, VEHICLE_NAME, VEHICLE_FCU_URL and VEHICLE_FIRMWARE
    source /etc/starling/vehicle.config
fi

function get_instance {
    # Attempt to get ordinal
    ORDINAL="${HOSTNAME##*-}"

    if [ -z "$ORDINAL" ]; then
        # Parsing ordinal failed, assume non-kubernetes container and use default of 0
        echo "0"
        return 2
    fi

    if ! [ "$ORDINAL" -ge 0 ] 2>/dev/null; then
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
    if [ $HAS_VEHICLE_CONFIG = true ]; then 
        echo "MAVROS_TGT_SYSTEM set to VEHICLE_MAVLINK_SYSID ($VEHICLE_MAVLINK_SYSID) from /etc/starling/vehicle.config"
        MAVROS_TGT_SYSTEM=$VEHICLE_MAVLINK_SYSID
    else
        set +e
        INSTANCE=$(get_instance)
        RESULT=$?
        MAVROS_TGT_SYSTEM=$((INSTANCE + 1))
        set -e
        if [ $RESULT -ne 0 ]; then
            echo "Could not parse ordinal, using default instance of 0. MAVROS_TGT_SYSTEM will be 1"
        else
            echo "MAVROS_TGT_SYSTEM set to $MAVROS_TGT_SYSTEM (from hostname: $HOSTNAME)"
        fi
    fi
elif (($MAVROS_TGT_SYSTEM >= 1 && $MAVROS_TGT_SYSTEM <= 255 )); then
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
    # If not set, then generate automatically
    if ! [ $HAS_VEHICLE_CONFIG = true ]; then
        # No vehicle.config, generate based on subparameters and the target system id dynamically
        set +e
        INSTANCE=$(get_instance)
        MAVROS_FCU_URL="$MAVROS_FCU_CONN://$MAVROS_FCU_IP:$((MAVROS_FCU_UDP_BASE + INSTANCE))@/"
        export MAVROS_FCU_URL=$MAVROS_FCU_URL
        set -e
        echo "MAVROS_FCU_URL automatically set to: $MAVROS_FCU_URL"
    else
        # Use value from vehicle.config
        export MAVROS_FCU_URL=$VEHICLE_FCU_URL
        echo "MAVROS_FCU_URL set to: $MAVROS_FCU_URL from vehicle.config"
    fi
else
    echo "MAVROS_FCU_URL was set to $MAVROS_FCU_URL"
fi
