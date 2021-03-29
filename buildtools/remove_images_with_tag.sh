#!/bin/bash

IMAGES=$(docker images --filter=reference="uobflightlabstarling/*:$1" | tail -n +2 | awk '{printf "%s:%s ", $1, $2}')

if ! [ -z "$IMAGES" ]; then
    docker rmi $IMAGES
fi
