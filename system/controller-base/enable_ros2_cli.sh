#!/bin/bash

echo "Enable CLI if ROS2 discovery server is in use. This is needed to properly use the ROS2 CLI tools as default discovery is not sufficient"
echo "For more information see this link: https://fast-dds.docs.eprosima.com/en/latest/fastdds/ros2/discovery_server/ros2_discovery_server.html#ros-2-introspection"

source /ros_entrypoint.sh

export FASTRTPS_DEFAULT_PROFILES_FILE=/super_client_configuration_file.xml
echo "Set FASTRTPS_DEFAULT_PROFILES_FILE to ${FASTRTPS_DEFAULT_PROFILES_FILE}"

ros2 daemon stop
ros2 daemon start
echo "ROS2 CLI tools ready to use" 