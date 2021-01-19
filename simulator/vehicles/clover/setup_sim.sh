#!/usr/bin/env bash

# Start model spawning script
/scripts/setup_model.sh &

# Launch the simulator
ros2 launch gazebo_ros gazebo.launch.py