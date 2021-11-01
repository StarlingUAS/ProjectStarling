# `starling-sim-base-core`

This image contains the Gazebo and gzweb installations that other Gazebo-based simulation images are built upon. It also
includes `gazebo-ros` to enable the link between ROS and Gazebo.

This image acts as a base to build other simulation environments on.

## `/ros.env.d`

To ease the expansion of this image, a mechanism has been built to enable expansion by volume mount. As part of the
entrypoint, the image will look for any folders in `/ros.env.d` that contain a `setup.bash` file. Any such file will be
sourced as part of the entrypoint. This allows for arbitrary expansion of the entrypoint by adding additional volume
mounts.

## Environment Variables

Name                  | Default Value                | Description
----------------------|------------------------------|------------
`ENABLE_VIRTUAL_FRAMEBUFFER` | true | Enables the `setup_display.sh` script and starts a virtual X server. Use for simulated vehicles which have cameras on them. 