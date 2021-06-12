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
elif (($AP_SYSID > 0 && $AP_SYSID <= 255 )); then
    echo "AP_SYSID setting as specified: $AP_SYSID"
    export AP_SYSID
else
    echo "AP_SYSID ($AP_SYSID) is invalid. Must either be set to 'ordinal' or number between 1 and 255 inclusive"
    exit 1
fi
echo "AP_SYSID is set to $AP_SYSID"

if [ "$VEHICLE" = copter ]; then
    MODEL=${MODEL:-quad}
    PARAM_FILE=copter.parm
else
    MODEL=${MODEL:-plane}
    PARAM_FILE=plane.parm
fi

PARAM_PATH=${PARAM_PATH:-/src/ardupilot/Tools/autotest/default_params}

# Insert the SYSID into the param file
echo SYSID_THISMAV $AP_SYSID >> ${PARAM_PATH}/${PARAM_FILE}

#if [ -z "$STARTPOSE" ]
#then
#  echo "Taking STARTPOSE from starts.txt file"
#  STARTPOSE=$(tail -n +$SYSID /home/pilot/app/starts.txt | head -n 1)
#fi
#echo "LAUNCH.SH: Start location will be $STARTPOSE"

exec /src/ardupilot/build/sitl/bin/ardu${VEHICLE} \
    --model=$MODEL \
    --defaults=/src/ardupilot/Tools/autotest/default_params/${PARAM_FILE}.parm