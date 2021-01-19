#!/bin/bash

# Setup ROS environment
source /opt/ros/foxy/setup.bash

# Setup environment for PX4's gazebo plugins
source /src/PX4-Autopilot/Tools/setup_gazebo.bash /src/PX4-Autopilot /src/PX4-Autopilot/build

# Run main command
exec "$@"
