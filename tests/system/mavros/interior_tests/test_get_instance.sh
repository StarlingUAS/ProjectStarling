#!/bin/bash

set -e

MAVROS_IMAGE=uobflightlabstarling/starling-mavros

# Test get_instance
docker run --rm $MAVROS_IMAGE bash -c 'source /ros_ws/mavros_setup.sh; INSTANCE=$(get_instance); [ $? -eq 2 ]; [ $INSTANCE -eq 0 ]'
docker run --rm $MAVROS_IMAGE bash -c 'source /ros_ws/mavros_setup.sh; INSTANCE=$(HOSTNAME=test-test get_instance); [ $? -eq 3 ]; [ $INSTANCE -eq 0 ]'
docker run --rm $MAVROS_IMAGE bash -c 'source /ros_ws/mavros_setup.sh; INSTANCE=$(HOSTNAME=test-5 get_instance); [ $? -eq 0 ]; [ $INSTANCE -eq 5 ]'
docker run --rm $MAVROS_IMAGE bash -c 'source /ros_ws/mavros_setup.sh; INSTANCE=$(PX4_INSTANCE_BASE=10 HOSTNAME=test-5 get_instance); [ $? -eq 0 ]; [ $INSTANCE -eq 15 ]'
