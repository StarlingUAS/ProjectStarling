#!/usr/bin/env bats

load ../../utils.bash
TEST_TAG=$(get_test_tag)
MAVROS_IMAGE=$(get_tagged_image uobflightlabstarling/starling-mavros)

@test "[$TEST_TAG] default VEHICLE_NAMESPACE" {
    docker run --rm $MAVROS_IMAGE bash -c '[ "$VEHICLE_NAMESPACE" == "vehicle_1" ]'
}

@test "[$TEST_TAG] explicit VEHICLE_NAMESPACE" {
    docker run --rm -e VEHICLE_NAMESPACE=vehicle_42 $MAVROS_IMAGE bash -c '[ "$VEHICLE_NAMESPACE" == "vehicle_42" ]'
}

@test "[$TEST_TAG] explicit VEHICLE_NAMESPACE with hyphens" {
    docker run --rm -e VEHICLE_NAMESPACE=vehicle-42 $MAVROS_IMAGE bash -c '[ "$VEHICLE_NAMESPACE" == "vehicle_42" ]'
}

@test "[$TEST_TAG] VEHICLE_NAMESPACE from hostname lookup" {
    docker run --rm -e HOSTNAME=test-42 $MAVROS_IMAGE bash -c '[ "$VEHICLE_NAMESPACE" == "vehicle_43" ]'
}
