#!/bin/bash
set -e

### build ros2 workspace
echo "Building ROS2 workspace dev_ws"
source /opt/ros/foxy/setup.bash
# rosdep update
cd ~/workspace/dev_ws
# rosdep install -i --from-path src --rosdistro foxy -y
colcon build