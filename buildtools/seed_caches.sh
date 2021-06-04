#!/bin/bash

# Seed the cache tag on DockerHub

export BAKE_CACHENAME_TO=cache
export BAKE_CACHE_REGISTRY=""

./build_local_multiplatform.sh

unset BAKE_CACHENAME_TO
unset BAKE_CACHE_REGISTRY
