#!/bin/bash

# Seed the cache tag on DockerHub

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

export BAKE_CACHETO_NAME=cache

${SCRIPT_DIR}/build_local_multiplatform.sh

unset BAKE_CACHETO_NAME
