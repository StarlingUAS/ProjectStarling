#!/bin/bash -e

REPO=${REPO:-}
TAG=${TAG:-}
PLATFORMS=${PLATFORMS:-linux/amd64}

# Build a container image
#
# Usage: build_image NAME DOCKERFILE CONTEXT
#
# NAME will be used as the name of the image
# DOCKERFILE is the path to the Dockerfile for the build
# CONTEXT is the path to the docker build context
#
# If REPO is set, name the image with the repo provided
# If TAG is set, add the tag to the image name
# If PLATFORMS is set, use that platform list for the build (Default: "linux/amd64")
# If PUSH is set, push the image to the provided repo
#
function build_image {
    NAME=$1
    DOCKERFILE=$2
    CONTEXT=$3
    if ! [ -z "${REPO}" -o -z "${PUSH}" ]; then
        SHOULD_PUSH=true
    fi
    docker buildx build ${SHOULD_PUSH:+--push} --platform ${PLATFORMS} -t ${REPO:+${REPO}/}${NAME}${TAG:+:${TAG}} -f ${DOCKERFILE} ${CONTEXT}
}

# Build a container image from a directory
#
# Usage: build_image_dir CONTEXT [NAME] [DOCKERFILE]
#
# CONTEXT is the path to the build context
# NAME is an optional override for the image name (Will have "starling-" prepended)
# DOCKERFILE is alternate Dockerfile name (relative to CONTEXT)
#
# Image name defaults to the basename of the directory with "starling-" prepended
#  e.g. system/mavros -> starling-mavros
# DOCKERFILE defaults to "Dockerfile"
#
function build_image_dir {
    CONTEXT=$1
    NAME=starling-${2:-$(basename ${CONTEXT})}
    DOCKERFILE=${CONTEXT}/${3:-Dockerfile}
    build_image ${NAME} ${DOCKERFILE} ${CONTEXT}
}

build_image_dir system/mavros
build_image_dir system/ui
build_image_dir system/controller-base

# Need to do this separately, not a pushed image but needed for build stages
docker buildx build --platform ${PLATFORMS} --target px4builder -t starling-px4-builder simulator/base/px4

build_image_dir simulator/base/core sim-base-core
build_image_dir simulator/base/px4 sim-base-px4
build_image_dir simulator/base/px4 sim-px4-sitl sitl.Dockerfile

build_image_dir simulator/ardupilot_ar ardupilot-sitl
