#!/bin/bash

# If MAVROS_TGT_SYSTEM is zero, then set instance id to IP address last digit
if [ "$MAVROS_TGT_SYSTEM" -eq "0" ]; then
    if [ -f "/etc/drone.config"]; then 
        echo "MAVROS_TGT_SYSTEM will be read from file not yet implemented, set to 1 for now"
        MAVROS_TGT_SYSTEM=1
    else
        IPADDR=`ifconfig eth0  | grep 'inet' | awk '{print $2}'`
        read A B C D <<< "${IPADDR//./ }"
        MAVROS_TGT_SYSTEM=$D;
        echo "MAVROS_TGT_SYSTEM set to $D from IP ADDRESS: $IPADDR (was zero and /etc/drone.config not found)"
    fi
else
    echo "MAVROS_TGT_SYSTEM setting as specified: $MAVROS_TGT_SYSTEM"
fi

export MAVROS_TGT_SYSTEM=$MAVROS_TGT_SYSTEM
