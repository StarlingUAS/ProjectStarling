#!/usr/bin/env bash

# Build base image for PX4 SITL
docker build -t starling-sim-base-px4 simulator/base/px4

#docker build -t starling-sim-base-ardupilot simulator/base/ardupilot

# Build overlay image for clover vehicle
docker build -t starling-sim-clover simulator/vehicles/clover
