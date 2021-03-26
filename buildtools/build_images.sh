#!/bin/bash -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

BUILDX_DRIVER="$(docker buildx inspect | grep Driver | awk '{print $2}')"

if [ ! $BUILDX_DRIVER = "docker-container" ]; then
    # Need to create a builder that supports multi-platform
    echo "Creating new builder instance"
    docker buildx create --use
fi

docker buildx bake --load -f $SCRIPT_DIR/docker-bake.hcl default
