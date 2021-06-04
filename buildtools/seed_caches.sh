#!/bin/bash

# Seed the cache tag on DockerHub

export BAKE_CACHETO_NAME=cache

./build_local_multiplatform.sh

unset BAKE_CACHETO_NAME
