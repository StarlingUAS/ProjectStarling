#!/bin/bash

#
# Builds the images and puts them in a local docker repository with the :latest tag
# Setting the VERSION environment variable will use that as tag instead. Additionally, this is passed to
#  the `.hcl` file to ensure that images that depend on other starling images are linked to the correct
#  version.
# Images are built for the platforms listed in the `bake.hcl` file.
# If the currently in-use builder does not use the docker-container driver, a new builder will be created
#  to build the images.
#

REGISTRY_NAME=starlingmp_registry
BUILDER_NAME=starlingmp_builder

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
ORIGINAL_DIR="$(pwd)"

function cleanup {
    cd $ORIGINAL_DIR
    echo "A builder ($BUILDER_NAME) and a registry ($REGISTRY_NAME) have been left running to speed up future runs"
    echo "These are likely to have large associated volumes. You may wish to clean them up: "
    echo "  $ docker container stop $REGISTRY_NAME"
    echo "  $ docker container rm $REGISTRY_NAME"
    echo "  $ docker buildx rm $BUILDER_NAME"
    echo "  $ docker system prune"
}
trap cleanup EXIT

LOCAL_REGISTRY_STATUS="$(docker container inspect $REGISTRY_NAME 2> /dev/null)"
if [ "$LOCAL_REGISTRY_STATUS" = "[]" ]; then
    # Need to start the registry
    set -e
    docker run -d -p 5000:5000 --restart=always --name starlingmp_registry registry:2
    set +e
fi

BUILDER_STATUS="$(docker buildx inspect $BUILDER_NAME 2> /dev/null)"
RESULT=$?
if [ $RESULT -eq 1 ]; then
    # Need to create a builder
    echo "Creating new builder instance"
    set -e
    docker buildx create --driver-opt network=host --name $BUILDER_NAME
    set +e
fi

cd $SCRIPT_DIR/..

set -e
BAKE_REGISTRY="localhost:5000/" docker buildx bake --builder $BUILDER_NAME --push -f $SCRIPT_DIR/docker-bake.hcl stage1
BAKE_REGISTRY="localhost:5000/" docker buildx bake --builder $BUILDER_NAME --push -f $SCRIPT_DIR/docker-bake.hcl stage2
BAKE_REGISTRY="localhost:5000/" docker buildx bake --builder $BUILDER_NAME --push -f $SCRIPT_DIR/docker-bake.hcl stage3

echo "Registry at localhost:5000 has your images"
echo "Pull the images by appending localhost:5000. e.g.:"
echo "  $ docker pull localhost:5000/uobflightlabstarling/rosbridge-suite"
