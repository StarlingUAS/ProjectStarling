#!/bin/bash
set -e

declare -a TAGS=(
    "latest"
    "nightly"
)

declare -a CTRS=(
    "uobflightlabstarling/starling-sim-iris"
    "uobflightlabstarling/example_controller_python"
    "uobflightlabstarling/starling-ui"
    "uobflightlabstarling/starling-sim-px4-sitl"
    "uobflightlabstarling/starling-sim-clover2"
    "uobflightlabstarling/starling-sim-base-px4"
    "uobflightlabstarling/starling-sim-base-core"
    "uobflightlabstarling/starling-controller-base"
    "uobflightlabstarling/starling-ardupilot-sitl"
)

for i in "${CTRS[@]}"
do
    for j in "${TAGS[@]}"
    do
        name="$i:$j"
        echo ">>>>>>  Checking $name"
        if [[ "$(docker images -q $name 2> /dev/null)" == "" ]]; then
        # do something
            echo "Image Exists Locally, attempting to pull"
            docker pull $name
        else
            echo "Image Doesn't Exist Locally, skipping"
        fi
    done
done
echo "Completed Update"