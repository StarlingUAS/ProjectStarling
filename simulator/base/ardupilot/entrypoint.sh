#!/bin/bash

# If AP_SYSID is "ordinal", then set AP_SYSID id to take AP_SYSID_BASE + ORDINAL + 1 from StatefulSet hostname
# Hostname is of the form '<stateful set name>-<ordinal>'
if [ "$AP_SYSID" == "ordinal" ]; then
    ORDINAL="${HOSTNAME##*-}"
    export AP_SYSID=$((AP_SYSID_BASE + ORDINAL + 1));
    echo "AP_SYSID was 'ordinal' therefore set to $AP_SYSID (from base: $AP_SYSID_BASE and hostname: $HOSTNAME)"
elif (($AP_SYSID > 0 && $AP_SYSID <= 255 )); then
    echo "AP_SYSID setting as specified: $AP_SYSID"
    export AP_SYSID
else
    echo "AP_SYSID (set to $AP_SYSID) is invalid, setting to 1. Must either be set to 'ordinal' or number between 1 and 255"
    export AP_SYSID=1
fi
echo "AP_SYSID is set to $AP_SYSID"

if [ "$VEHICLE" = copter ]; then
    MODEL=quad
    PARAM_FILE=copter
else
    MODEL=plane
    PARAM_FILE=plane
fi

# Insert the SYSID into the param file
echo SYSID_THISMAV $AP_SYSID >> /src/ardupilot/Tools/autotest/default_params/${PARAM_FILE}.parm

#if [ -z "$STARTPOSE" ]
#then
#  echo "Taking STARTPOSE from starts.txt file"
#  STARTPOSE=$(tail -n +$SYSID /home/pilot/app/starts.txt | head -n 1)
#fi
#echo "LAUNCH.SH: Start location will be $STARTPOSE"

exec /src/ardupilot/build/sitl/bin/ardu${VEHICLE} \
    --model=$MODEL \
    --defaults=/src/ardupilot/Tools/autotest/default_params/${PARAM_FILE}.parm