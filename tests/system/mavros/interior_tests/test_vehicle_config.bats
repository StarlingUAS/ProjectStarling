#!/usr/bin/env bats

MAVROS_IMAGE=uobflightlabstarling/starling-mavros

@test "HAS_VEHICLE_CONFIG=false without file" {
    docker run --rm $MAVROS_IMAGE bash -c 'source /ros_ws/mavros_setup.sh; [ "$HAS_VEHICLE_CONFIG" == "false" ]'
}

@test "HAS_VEHICLE_CONFIG=true with file" {
    docker run --rm -v $(pwd)/resources/example_vehicle.config:/etc/starling/vehicle.config $MAVROS_IMAGE bash -c 'source /ros_ws/mavros_setup.sh; [ "$HAS_VEHICLE_CONFIG" == "true" ]'
}
