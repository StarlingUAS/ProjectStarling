#!/usr/bin/env bats

load ../../utils.bash
TEST_TAG=$(get_test_tag)
MAVROS_IMAGE=$(get_tagged_image uobflightlabstarling/starling-mavros)

function setup_file() {
    export CONTAINER_ID=$(docker run -d --rm $MAVROS_IMAGE)
}

@test "[$TEST_TAG] mavros fcu_url set" {
    url="$(docker exec $CONTAINER_ID bash -c 'source /opt/ros/$ROS1_DISTRO/setup.bash; sleep 2; rosparam get /vehicle_1/mavros/fcu_url' | tail -n1)"
    [ "$url" == "udp://127.0.0.1:14830@/?ids=1,240" ]
}

function teardown_file() {
    docker kill $CONTAINER_ID
}