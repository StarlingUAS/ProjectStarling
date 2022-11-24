#!/usr/bin/env bats

EXAMPLE_CONFIG_DIR=$BATS_TEST_DIRNAME/resources/starling/

load ../../utils.bash
TEST_TAG=$(get_test_tag)
MAVROS_IMAGE=$(get_tagged_image starling-mavros)

@test "[$TEST_TAG] HAS_VEHICLE_CONFIG=false without file" {
    docker run --rm $MAVROS_IMAGE bash -c 'source /ros_ws/mavros_setup.sh; [ "$HAS_VEHICLE_CONFIG" == "false" ]'
}

@test "[$TEST_TAG] HAS_VEHICLE_CONFIG=true with file" {
    docker run --rm -v $EXAMPLE_CONFIG_DIR:/etc/starling/ $MAVROS_IMAGE bash -c 'source /ros_ws/mavros_setup.sh; [ "$HAS_VEHICLE_CONFIG" == "true" ]'
}
