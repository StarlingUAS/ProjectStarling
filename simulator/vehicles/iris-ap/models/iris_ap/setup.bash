#!/bin/bash
SCRIPT_DIR="$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

XACRO_PATH=${SCRIPT_DIR}/model.sdf.xacro
MODEL_PATH=/ardupilot_gazebo/models/iris_with_ardupilot/model.sdf

xacro ${XACRO_PATH} \
    ros_namespace:=${VEHICLE_NAMESPACE} \
    fdm_addr:=${AP_SITL_ADDRESS} \
    listen_addr:=0.0.0.0 \
    -o ${MODEL_PATH}

echo "Written iris model to ${MODEL_PATH}"
