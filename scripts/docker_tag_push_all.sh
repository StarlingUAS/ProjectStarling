#!/bin/bash
set -e

HUBNAME="uobflightlabstarling"

declare -a LOCAL_CTRS=(
    "starling-sim-iris"
    "example_controller_python"
    "starling-ui"
    "starling-sim-px4-sitl"
    "starling-sim-clover2"
    "starling-sim-base-px4"
    "starling-sim-base-core"
    "starling-controller-base"
    "starling-ardupilot-sitl"
)

for i in "${LOCAL_CTRS[@]}"
do
    echo ">>>>>>  TAG AND PUSH $i"
    CTRNAME="$i"
    docker tag $CTRNAME $HUBNAME/$CTRNAME
    docker push $HUBNAME/$CTRNAME
    echo ">>>>>>  PUSHED $HUBNAME/$CTRNAME"
done