#!/usr/bin/env bats

MAVROS_IMAGE=uobflightlabstarling/starling-mavros

load ../../utils.bash
TEST_TAG=$(get_test_tag)

# Test get_instance
@test "[$TEST_TAG] get_instance in default state" {
    docker run --rm $MAVROS_IMAGE bash -c 'source /ros_ws/mavros_setup.sh; INSTANCE=$(get_instance); [ $? -ne 0 ] && [ $INSTANCE -eq 0 ]'
}

@test "[$TEST_TAG] get_instance with unparsable hostname" {
    docker run --rm -e HOSTNAME=test- $MAVROS_IMAGE bash -c 'source /ros_ws/mavros_setup.sh; INSTANCE=$(get_instance); [ $? -eq 2 ] && [ $INSTANCE -eq 0 ]'
}

@test "[$TEST_TAG] get_instance with unparsable ordinal" {
    docker run --rm -e HOSTNAME=test-test $MAVROS_IMAGE bash -c 'source /ros_ws/mavros_setup.sh; INSTANCE=$(get_instance); [ $? -eq 3 ] && [ $INSTANCE -eq 0 ]'
}

@test "[$TEST_TAG] get_instance with parseable ordinal" {
    docker run --rm -e HOSTNAME=test-5 $MAVROS_IMAGE bash -c 'source /ros_ws/mavros_setup.sh; INSTANCE=$(get_instance); [ $? -eq 0 ] && [ $INSTANCE -eq 5 ]'
}

@test "[$TEST_TAG] get_instance with parseable hostname and PX4_INSTANCE_BASE" {
    docker run --rm -e HOSTNAME=test-5 -e PX4_INSTANCE_BASE=10 $MAVROS_IMAGE bash -c 'source /ros_ws/mavros_setup.sh; INSTANCE=$(get_instance); [ $? -eq 0 ] && [ $INSTANCE -eq 15 ]'
}
