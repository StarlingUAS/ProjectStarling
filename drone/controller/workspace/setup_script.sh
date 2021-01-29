#!/bin/bash

rosdep update

### build ros2 workspace
echo "Building ROS2 workspace dev_ws"
source /opt/ros/foxy/setup.bash
cd dev_ws
rosdep install -i --from-path src --rosdistro foxy -y
colcon build
cd ../

### build ros1 workspace
echo "Building ROS1 workspace catkin_ws"
source /opt/ros/noetic/setup.bash
cd catkin_ws
rosdep install -i --from-path src --rosdistro noetic -y
catkin_make
cd ../

### source ros2 workspace
echo "Sourcing ros2 workspace dev_ws"
source /opt/ros/foxy/setup.bash
source dev_ws/install/local_setup.bash

### or source ros1 workspace
# source /opt/ros/noetic/setup.bash
# source catkin_ws/devel/setup.bash