#!/usr/bin/env bats

EXAMPLE_CONFIG_DIR=$BATS_TEST_DIRNAME/resources/starling/

load ../../utils.bash
TEST_TAG=$(get_test_tag)
MAVROS_IMAGE=$(get_tagged_image starling-mavros)

@test "[$TEST_TAG] explicit MAVROS_TGT_SYSTEM" {
    docker run --rm -e MAVROS_TGT_SYSTEM=123 $MAVROS_IMAGE bash -c '[ "$MAVROS_TGT_SYSTEM" -eq 123 ]'
}

@test "[$TEST_TAG] explicit MAVROS_TGT_SYSTEM too low" {
    docker run --rm -e MAVROS_TGT_SYSTEM=0 $MAVROS_IMAGE bash -c '[ "$MAVROS_TGT_SYSTEM" -eq 1 ]'
}

@test "[$TEST_TAG] explicit MAVROS_TGT_SYSTEM too high" {
    docker run --rm -e MAVROS_TGT_SYSTEM=256 $MAVROS_IMAGE bash -c '[ "$MAVROS_TGT_SYSTEM" -eq 1 ]'
}

@test "[$TEST_TAG] MAVROS_TGT_SYSTEM with sysid from vehicle config" {
    docker run --rm -v $EXAMPLE_CONFIG_DIR:/etc/starling/ $MAVROS_IMAGE bash -c '[ "$MAVROS_TGT_SYSTEM" -eq 23 ]'
}

@test "[$TEST_TAG] MAVROS_TGT_SYSTEM with sysid from hostname" {
    docker run --rm -e HOSTNAME=test-5 $MAVROS_IMAGE bash -c '[ "$MAVROS_TGT_SYSTEM" -eq 6 ]'
}

@test "[$TEST_TAG] MAVROS_TGT_SYSTEM with sysid from hostname with unparseable ordinal" {
    docker run --rm -e HOSTNAME=test $MAVROS_IMAGE bash -c '[ "$MAVROS_TGT_SYSTEM" -eq 1 ]'
}

@test "[$TEST_TAG] MAVROS_TGT_SYSTEM with sysid from hostname and instance base" {
    docker run --rm -e HOSTNAME=test-5 -e PX4_INSTANCE_BASE=10 $MAVROS_IMAGE bash -c '[ "$MAVROS_TGT_SYSTEM" -eq 16 ]'
}
