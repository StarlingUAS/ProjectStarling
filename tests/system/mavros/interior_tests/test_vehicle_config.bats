#!/usr/bin/env bats

MAVROS_IMAGE=uobflightlabstarling/starling-mavros
EXAMPLE_CONFIG_PATH=$BATS_TEST_DIRNAME/resources/example_vehicle.config

@test "HAS_VEHICLE_CONFIG=false without file" {
    docker run --rm $MAVROS_IMAGE bash -c 'source /ros_ws/mavros_setup.sh; [ "$HAS_VEHICLE_CONFIG" == "false" ]'
}

@test "HAS_VEHICLE_CONFIG=true with file" {
    docker run --rm -v $EXAMPLE_CONFIG_PATH:/etc/starling/vehicle.config $MAVROS_IMAGE bash -c 'source /ros_ws/mavros_setup.sh; [ "$HAS_VEHICLE_CONFIG" == "true" ]'
}
