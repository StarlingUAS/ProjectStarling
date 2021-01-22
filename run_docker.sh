#!/usr/bin/env bash

xhost +

docker run -it \
    -p 18570:18570/udp \
    -p 14580:14580/udp \
    -p 14280:14280/udp \
    -p 8080:8080 \
    starling-sim-clover2 "$@"
