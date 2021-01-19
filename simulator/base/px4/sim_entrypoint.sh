#!/bin/bash

# Setup environment for PX4's gazebo plugins
source Tools/setup_gazebo.bash /src/PX4-Autopilot /src/PX4-Autopilot/build

# Run main command
exec "$@"
