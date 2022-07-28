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
        # TODO: Add validity check for SYSID
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
    TGT_SYSTEM_SET=true
else
    # TODO: Fail early here
    echo "MAVROS_TGT_SYSTEM (set to $MAVROS_TGT_SYSTEM) is invalid, setting to 1. Must either be set to 'auto' where it will either look  or number between 1 and 256"
    MAVROS_TGT_SYSTEM=1
    TGT_SYSTEM_SET=true
fi

export MAVROS_TGT_SYSTEM=$MAVROS_TGT_SYSTEM


if [ ! -v $VEHICLE_NAMESPACE ]; then
    # If set Ensure VEHICLE_NAMESPACE is a valid topic name
    # Replace all '-' with '_'
    export VEHICLE_NAMESPACE=${VEHICLE_NAMESPACE//-/_}
    echo "VEHICLE_NAMESPACE setting to $VEHICLE_NAMESPACE"
else
    export VEHICLE_NAMESPACE="vehicle_$MAVROS_TGT_SYSTEM"
    echo "VEHICLE_NAMESPACE not set, default to auto generated default based on mavros target ($MAVROS_TGT_SYSTEM) of $VEHICLE_NAMESPACE"
fi


ZENOH_ALLOW_TOPICS="mavros|$ZENOH_ALLOW_TOPICS"

export RUST_LOG=info

if [ ! -v $SERVER ]; then
    echo "Running Zenoh on Server"
    exec zenoh-bridge-dds \
        -d 0 \
        -l tcp/0.0.0.0:7447
else
    echo "Running Zenoh on Board Vehicle $VEHICLE_MAVLINK_SYSID, with ROS_DOMAIN_ID=0 and scope set to $VEHICLE_MAVLINK_SYSID"
    echo "Allowing the following topics: '"$ZENOH_ALLOW_TOPICS"'"
    exec zenoh-bridge-dds \
        -d 0\
        -l tcp/0.0.0.0:7447 \
        --allow "'"$ZENOH_ALLOW_TOPICS"'" \
        --scope "/"$VEHICLE_NAMESPACE""
fi
