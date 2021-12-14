#!/bin/bash

set -e

MAVROS_IMAGE=uobflightlabstarling/starling-mavros

# Test HAS_VEHICLE_CONFIG
docker run --rm $MAVROS_IMAGE bash -c 'source /ros_ws/mavros_setup.sh; [ "$HAS_VEHICLE_CONFIG" == "false" ]'
docker run --rm $MAVROS_IMAGE bash -c 'mkdir -p /etc/starling; touch /etc/starling/vehicle.config; source /ros_ws/mavros_setup.sh; [ "$HAS_VEHICLE_CONFIG" == "true" ]'
