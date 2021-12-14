#!/usr/bin/env bats

MAVROS_IMAGE=uobflightlabstarling/starling-mavros
EXAMPLE_CONFIG_PATH=$BATS_TEST_DIRNAME/resources/example_vehicle.config

@test "explicit MAVROS_FCU_URL" {
    docker run --rm -e MAVROS_FCU_URL="serial:///dev/test/" $MAVROS_IMAGE bash -c '[ "$MAVROS_FCU_URL" == "serial:///dev/test/" ]'
}

@test "MAVROS_FCU_URL with instance from hostname" {
    docker run --rm -e HOSTNAME=test-5 $MAVROS_IMAGE bash -c '[ "$MAVROS_FCU_URL" == "udp://127.0.0.1:14835@/" ]'
}

@test "MAVROS_FCU_URL with FCU_CONN, FCU_IP and FCU_UDP_BASE and instance from hostname" {
    docker run --rm \
        -e HOSTNAME=test-5 \
        -e MAVROS_FCU_CONN=tcp \
        -e MAVROS_FCU_IP=10.0.0.5 \
        -e MAVROS_FCU_UDP_BASE=5760 \
        $MAVROS_IMAGE bash -c '[ "$MAVROS_FCU_URL" == "tcp://10.0.0.5:5765@/" ]'
}

@test "MAVROS_FCU_URL from vehicle config" {
    docker run --rm -v $EXAMPLE_CONFIG_PATH:/etc/starling/vehicle.config $MAVROS_IMAGE bash -c '[ "$MAVROS_FCU_URL" == "serial:///dev/px4fmu:115200" ]'
}
