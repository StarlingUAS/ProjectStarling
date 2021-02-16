#!/bin/bash

# MAVROS_TGT_SYSTEM may come in as an IP Address
for i in ${MAVROS_TGT_SYSTEM} ; do
    if [[ $i == *.* ]]; then
        read A B C D <<<"${MAVROS_TGT_SYSTEM//./ }"
        echo "MAVROS TGT IS IN IP ADDRESS FORM (${MAVROS_TGT_SYSTEM}), WILL EXTRACT FINAL DIGIT FOR USE"
        export MAVROS_TGT_SYSTEM=$D;
        break;
    fi
done
