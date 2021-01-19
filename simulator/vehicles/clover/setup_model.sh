#!/usr/bin/env bash

#<!-- Toggleable model parameters -->
#<!-- Main camera -->
#<arg name="main_camera" default="true"/>
#<!-- Slow simulation down to maintain camera rate -->
#<arg name="maintain_camera_rate" default="false"/>
#<arg name="rangefinder" default="true"/>
#<arg name="led" default="true"/>
#<arg name="gps" default="true"/>
#<!-- Use physics parameters from CAD programs -->
#<arg name="use_clover_physics" default="false"/>

ros2 run xacro xacro /src/clover/clover_description/urdf/clover/clover4.xacro \
    main_camera:=false \
    rangefinder:=false \
    led:=false \
    gps:=false \
    maintain_camera_rate:=false \
    use_clover_physics:=false \
    | ros2 run gazebo_ros spawn_entity.py -entity clover -x 0 -y 0 -z 0 -stdin
