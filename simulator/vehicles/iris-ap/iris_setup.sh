#!/bin/bash

if [ ! -z "${AP_SITL_HOST}" ]; then
    # AP_SITL_HOST has been set, use it to set AP_SITL_ADDRESS
    AP_SITL_ADDRESS="$(getent hosts ${AP_SITL_HOST} | cut -d ' ' -f1)"
    if [ -z "${AP_SITL_ADDRESS}" ]; then
        # Address lookup failed
        echo "Error: Failed to lookup IP address for host '${AP_SITL_HOST}'"
        exit 1
    fi
fi

XACRO_PATH=/ros.env.d/iris_ap/model.sdf.xacro
MODEL_PATH=/ardupilot_gazebo/models/iris_with_ardupilot/model.sdf

xacro ${XACRO_PATH} fdm_addr:=${AP_SITL_ADDRESS} > ${MODEL_PATH}
