#!/bin/bash

# If AP_SYSID is "ordinal", then set AP_SYSID id to take AP_SYSID_BASE + ORDINAL from StatefulSet hostname
# Hostname is of the form '<stateful set name>-<ordinal>'
if [ "$AP_SYSID" == "ordinal" ]; then
    ORDINAL="${HOSTNAME##*-}"
    AP_SYSID=$((AP_SYSID_BASE + ORDINAL));
    if (($AP_SYSID > 0 && $AP_SYSID <= 255 )); then
        echo "AP_SYSID was 'ordinal' therefore set to $AP_SYSID (from base: $AP_SYSID_BASE and hostname: $HOSTNAME)"
        export AP_SYSID
    else
        echo "AP_SYSID was 'ordinal' but result ($AP_SYSID) was not valid (from base: $AP_SYSID_BASE and hostname: $HOSTNAME)"
        exit 1
    fi
elif [ "$AP_SYSID" == "ip" ]; then
    AP_SYSID=$(hostname -i | cut -d'.' -f4)
    echo "AP_SYSID was 'ip' therefore set to $AP_SYSID (from IP: $(hostname -i))"
    export AP_SYSID
elif (($AP_SYSID > 0 && $AP_SYSID <= 255 )); then
    echo "AP_SYSID setting as specified: $AP_SYSID"
    export AP_SYSID
else
    echo "AP_SYSID ($AP_SYSID) is invalid. Must either be set to 'ordinal', 'ip' or number between 1 and 255 inclusive"
    exit 1
fi
echo "AP_SYSID is set to $AP_SYSID"

if [ ! -z "${AP_SITL_HOST}" ]; then
    # AP_SITL_HOST has been set, use it to set AP_SITL_ADDRESS
    AP_SITL_ADDRESS="$(getent hosts ${AP_SITL_HOST} | cut -d ' ' -f1)"
    if [ -z "${AP_SITL_ADDRESS}" ]; then
        # Address lookup failed
        echo "Error: Failed to lookup IP address for host '${AP_SITL_HOST}'"
        exit 1
    fi
fi

if [ ! -v $VEHICLE_NAMESPACE ]; then
    # If set Ensure VEHICLE_NAMESPACE is a valid topic name
    # Replace all '-' with '_'
    export VEHICLE_NAMESPACE=${VEHICLE_NAMESPACE//-/_}
    echo "VEHICLE_NAMESPACE setting to $VEHICLE_NAMESPACE"
else
    export VEHICLE_NAMESPACE="vehicle_$AP_SYSID"
    echo "VEHICLE_NAMESPACE not set, default to auto generated default based on ardupilot sysid target ($AP_SYSID) of $VEHICLE_NAMESPACE"
fi