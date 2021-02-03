#!/bin/bash

# SYSID may come in as an IP Address
for i in ${SYSID} ; do
    if [[ $i == *.* ]]; then
        read A B C D <<<"${SYSID//./ }"
        echo "SYSID IS IN IP ADDRESS FORM (${SYSID}), WILL EXTRACT FINAL DIGIT FOR USE"
        export SYSID=$D;
        break;
    fi
done
