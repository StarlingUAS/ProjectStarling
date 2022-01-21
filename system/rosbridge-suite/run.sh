#!/bin/bash
set -e

# Check for mounted msgs
if [ ! -d "${MSGS_WS}" ]; then
    echo "No external messages mounted at ${MSGS_WS}"
elif [ -d "${MSGS_WS}/install" ]; then
    echo "Messages already built at ${MSGS_WS}/install, not building"
else
    echo "External messages detected at ${MSGS_WS}, building..."
    colcon build --base-paths "${MSGS_WS}" --build-base "${MSGS_WS}/build" --install-base "${MSGS_WS}/install"
    rm -r "${MSGS_WS}/build"
    source "${MSGS_WS}/install/setup.bash"
fi

ros2 launch /ros_ws/rosbridge_websocket_launch.xml port:=${ROSBRIDGE_PORT}