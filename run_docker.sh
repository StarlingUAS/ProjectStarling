#!/usr/bin/env bash

xhost +

docker run -it --privileged \
    --env=LOCAL_USER_ID="$(id -u)" \
    -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
    -e DISPLAY=:0 \
    -e NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics \
    -p 14570:14570/udp \
    --gpus all \
    starling-sim-clover2 "$@"
