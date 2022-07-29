# `starling-controller-base`

This image contains the minimal dependency requirements for a user to implement their own controller. In particular it contains a number of automated steps to aid in setup.

This image acts as a base to build controllers

## `/ros.env.d`

To ease the expansion of this image, a mechanism has been built to enable expansion by volume mount. 

As part of the
entrypoint, the image will look for any folders in `/ros.env.d` that contain a `setup.bash` file. 

Any such file will be
sourced as part of the entrypoint. This allows for arbitrary expansion of the entrypoint by adding additional volume
mounts.

## Environment Variables

Name                  | Default Value                | Description
----------------------|------------------------------|------------
`USE_SIMULATED_TIME` | false | A variable which can be used in ros2 launch scripts to toggle the `use_sim_time` parameter of a ros node. 