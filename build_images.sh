#!/usr/bin/env bash

# Build core image for gazebo, gzweb & ros2
docker build -t starling-sim-base-core simulator/base/core

# Build base image for PX4 SITL
docker build -t starling-sim-base-px4 simulator/base/px4

#docker build -t starling-sim-base-ardupilot simulator/base/ardupilot

# Build overlay image for clover vehicle
#docker build -t starling-sim-clover simulator/vehicles/clover

# Build overlay image for clover2 vehicle
docker build -t starling-sim-clover2 simulator/vehicles/clover2

# Build system layer image
#docker build -t starling-mavros system/mavros
