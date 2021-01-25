#!/usr/bin/env bash

docker run -it \
    -p 8080:8080 \
    starling-sim-clover2 "$@"
