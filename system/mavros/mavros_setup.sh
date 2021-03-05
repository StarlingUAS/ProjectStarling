#!/bin/bash

# If MAVROS_TGT_SYSTEM is zero, then set instance id to take MAVROS_TGT_SYSTEM_BASE + ORDINAL-1 from StatefulSet hostname
# Hostname is of the form '<stateful set name>-<ordinal>'
if [ "$MAVROS_TGT_SYSTEM" -eq "0" ]; then
    if [ -f "/etc/drone.config" ]; then 
        echo "MAVROS_TGT_SYSTEM will be read from file not yet implemented, set to 1 for now"
        MAVROS_TGT_SYSTEM=1
    else
        ORDINAL="${HOSTNAME##*-}"
        MAVROS_TGT_SYSTEM=$((MAVROS_TGT_SYSTEM_BASE + ORDINAL));
        echo "MAVROS_TGT_SYSTEM set to $MAVROS_TGT_SYSTEM (from hostname: $HOSTNAME), was set to zero and /etc/drone.config not found"
    fi
else
    echo "MAVROS_TGT_SYSTEM setting as specified: $MAVROS_TGT_SYSTEM"
fi

export MAVROS_TGT_SYSTEM=$MAVROS_TGT_SYSTEM

# Ensure VEHICLE_NAMESPACE is a valid topic name
# Replace all '-' with '_'
export VEHICLE_NAMESPACE=${VEHICLE_NAMESPACE//-/_}
