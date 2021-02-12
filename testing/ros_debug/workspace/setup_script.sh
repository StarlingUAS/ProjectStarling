#!/bin/bash

rosdep update

### build ros2 workspace
echo "Building ROS2 workspace dev_ws"
source /opt/ros/foxy/setup.bash
cd dev_ws
rosdep install -i --from-path src --rosdistro foxy -y
colcon build
cd ../

### source ros2 workspace
echo "Sourcing ros2 workspace dev_ws"
source /opt/ros/foxy/setup.bash
source dev_ws/install/local_setup.bash