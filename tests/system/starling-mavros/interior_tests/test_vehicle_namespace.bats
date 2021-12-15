#!/usr/bin/env bats

MAVROS_IMAGE=uobflightlabstarling/starling-mavros

@test "default VEHICLE_NAMESPACE" {
    docker run --rm $MAVROS_IMAGE bash -c '[ "$VEHICLE_NAMESPACE" == "vehicle_1" ]'
}

@test "explicit VEHICLE_NAMESPACE" {
    docker run --rm -e VEHICLE_NAMESPACE=vehicle_42 $MAVROS_IMAGE bash -c '[ "$VEHICLE_NAMESPACE" == "vehicle_42" ]'
}

@test "explicit VEHICLE_NAMESPACE with hyphens" {
    docker run --rm -e VEHICLE_NAMESPACE=vehicle-42 $MAVROS_IMAGE bash -c '[ "$VEHICLE_NAMESPACE" == "vehicle_42" ]'
}

@test "VEHICLE_NAMESPACE from hostname lookup" {
    docker run --rm -e HOSTNAME=test-42 $MAVROS_IMAGE bash -c '[ "$VEHICLE_NAMESPACE" == "vehicle_43" ]'
}
