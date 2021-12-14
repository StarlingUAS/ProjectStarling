#!/usr/bin/env bats

MAVROS_IMAGE=uobflightlabstarling/starling-mavros

@test "explicit MAVROS_TGT_SYSTEM" {
    docker run --rm -e MAVROS_TGT_SYSTEM=123 $MAVROS_IMAGE bash -c '[ "$MAVROS_TGT_SYSTEM" -eq 123 ]'
}

@test "explicit MAVROS_TGT_SYSTEM too low" {
    docker run --rm -e MAVROS_TGT_SYSTEM=0 $MAVROS_IMAGE bash -c 'MAVROS_TGT_SYSTEM=0 [ "$MAVROS_TGT_SYSTEM" -eq 123 ]'
}

@test "explicit MAVROS_TGT_SYSTEM too high" {
    docker run --rm -e MAVROS_TGT_SYSTEM=300 $MAVROS_IMAGE bash -c 'MAVROS_TGT_SYSTEM=0 [ "$MAVROS_TGT_SYSTEM" -eq 123 ]'
}

@test "MAVROS_TGT_SYSTEM with sysid from vehicle config" {
    docker run --rm -v $(pwd)/resources/example_vehicle.config:/etc/starling/vehicle.config $MAVROS_IMAGE bash -c '[ "$MAVROS_TGT_SYSTEM" -eq 23 ]'
}

@test "MAVROS_TGT_SYSTEM with sysid from hostname" {
    docker run --rm -e HOSTNAME=test-5 $MAVROS_IMAGE bash -c '[ "$MAVROS_TGT_SYSTEM" -eq 6 ]'
}

@test "MAVROS_TGT_SYSTEM with sysid from hostname with unparseable ordinal" {
    docker run --rm -e HOSTNAME=test $MAVROS_IMAGE bash -c '[ "$MAVROS_TGT_SYSTEM" -eq 1 ]'
}

@test "MAVROS_TGT_SYSTEM with sysid from hostname and instance base" {
    docker run --rm -e HOSTNAME=test-5 -e PX4_INSTANCE_BASE=10 $MAVROS_IMAGE bash -c '[ "$MAVROS_TGT_SYSTEM" -eq 16 ]'
}
