#!/bin/bash -e
set -e

#
# Builds the images and puts them in the local docker image store with the :latest tag
# Setting the VERSION environment variable will use that as tag instead. Additionally, this is passed to
#  the `.hcl` file to ensure that images that depend on other starling images are linked to the correct
#  version.
# Images are only built for the primary host platform (The first listed in `docker buildx inspect default`)
# If the currently in-use builder does not use the docker driver, images are built using the `default` builder. 
#

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
ORIGINAL_DIR="$(pwd)"

function cleanup {
    cd $ORIGINAL_DIR
}
trap cleanup EXIT

BUILDX_DRIVER="$(docker buildx inspect | grep Driver | awk '{print $2}')"
BUILDX_HOST_PLATFORM="$(docker buildx inspect default | sed -nE 's/^Platforms: ([^,]*),.*$/\1/p')"

if [ ! $BUILDX_DRIVER = "docker" ]; then
    # Need to use the docker driver for this script
    BUILDER_ARGS="--builder default"
else
    BUILDER_ARGS=""
fi


cd $SCRIPT_DIR/..

docker buildx bake $BUILDER_ARGS --set *.platform=$BUILDX_HOST_PLATFORM --load -f $SCRIPT_DIR/docker-bake.hcl stage1
docker buildx bake $BUILDER_ARGS --set *.platform=$BUILDX_HOST_PLATFORM --load -f $SCRIPT_DIR/docker-bake.hcl stage2
docker buildx bake $BUILDER_ARGS --set *.platform=$BUILDX_HOST_PLATFORM --load -f $SCRIPT_DIR/docker-bake.hcl stage3
