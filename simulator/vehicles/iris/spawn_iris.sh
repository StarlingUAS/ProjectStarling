#!/bin/bash

ros2 launch launch/iris.launch.xml spawn_only:=true

if [[ $IGNORE_FAILURE ]]; then
    exit 0
fi