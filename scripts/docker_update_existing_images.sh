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
cmd='docker images --format {{.Repository}}:{{.Tag}}'
IFS=$'\n' read -r -d '' -a CTRS < <(  $cmd && printf '\0' )

echo "Updating Local Images"

for i in "${CTRS[@]}"
do
    echo ">>>>>>  Checking $i"
    ! docker pull $i
done

echo "Completed Update"