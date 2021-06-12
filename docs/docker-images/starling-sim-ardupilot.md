# `starling-sim-ardu*`

There are two images currently built: `starling-sim-arducopter` and `starling-sim-arduplane`. These are built against
the relevant stable branch: `ArduCopter-stable` and `ArduPlane-stable` respectively.

The SITL starts up with a TCP server listening on port 5760. It will not begin simulating the vehicle until a connection
is made to this port. Currently, the SITL's built-in simulator is used for both `copter` and `plane` images.

## Environment Variables

Name                  | Default Value                | Description
----------------------|------------------------------|------------
`AP_SYSID`            | 1                            | MAVLink system ID to be used by the SITL. Can also be set to `"ordinal"`
`AP_SYSID_BASE`       | 1                            | Base system ID for ordinal-based generation
`AP_VEHICLE`          | {from build arg}             | Which vehicle is being used. Either `"copter"` or `"plane"`
`AP_MODEL`            | {null}                       | Alternate model argument. Set to __override__ default model
`AP_PARAM_PATH`       | {null}                       | Alternate path to parameter file. Set to __override__ default path
`AP_PARAM_FILE`       | {null}                       | Alternate parameter file name. Set to __override__ default parameter file
`AP_HOME`             | 51.4235413,-2.6708488,50,250 | Start location for SITL
`AP_OFFSET_X`         | 0                            | Start location x offset for SITL
`AP_OFFSET_Y`         | 0                            | Start location y offset for SITL

### `AP_SYSID`

Sets the MAVLink system ID to be used by the SITL. This must either be a value between 1 and 255 inclusive or set to
`"ordinal"`. Other values will cause the container to exit.

When set to `"ordinal`, the entrypoint script will attempt to retrieve a 0-indexed ordinal from the end of the
container's hostname. The hostnames should be of the form `${HOSTNAME}-${ORDINAL}`, *e.g.* `sitl-0`, `sitl-1`, `sitl-3`.
This behaviour supports deployment of this image as part of a Kubernetes StatefulSet.

At present, the system ID is set by appending it to the parameter file. If a custom parameter file is supplied, it
should not contain the `SYSID_THISMAV` parameter.

### `AP_SYSID_BASE`

This variable only affects the container when using the ordinal-based generation outlined above. The value set here will
be added to the ordinal from the hostname to derive the MAVLink system ID. If the resultant value is not a valid system
ID, *i.e.* it is not between 1 and 255 inclusive, the container will exit.

### `AP_VEHICLE`

By default this will be either `"copter"` or `"plane"` and is derived from the image's arguments at build time. This
argument is used to set which executable is used and should not be overriden. It is also used to set defaults for the
`MODEL` and `PARAM_FILE` variables unless they are overridden.

### `AP_MODEL`

Used as the value of the `--model` argument to the ArduPilot SITL binary. For `copter` images, this is set to `quad`.
For `plane` images, this is set to `plane`.

### `AP_PARAM_PATH`

The path to the folder containing the parameter file. If left blank, the in-source folder will be used:
`Tools/autotest/default_params`. This can be changed to allow for supplying a custom parameter file through a volume
mount.

At present, the system ID is set by appending it to the parameter file. If a custom parameter file is supplied, it
should not contain the `SYSID_THISMAV` parameter.

### `AP_PARAM_FILE`

The name of the parameter file to use. For `copter` images, this is set to `copter.parm`. For `plane` images, this is
set to `plane.parm`. This can be changed to use either one of the in-source default parameter files, or to use a custom
parameter file.

At present, the system ID is set by appending it to the parameter file. If a custom parameter file is supplied, it
should not contain the `SYSID_THISMAV` parameter.

### `AP_HOME`, `AP_OFFSET_X` & `AP_OFFSET_Y`

`AP_HOME` sets the start location for the SITL. Format is `{Latitude},{Longitude},{Altitude},{Heading}`

`AP_OFFSET_X` and `AP_OFFSET_Y` are body frame offsets in metres from `AP_HOME` for this vehicle. This allows a grid
of vehicles to be created using a single `AP_HOME` while varying the offsets.

## Dockerfile Build Arguments

### `VEHICLE`

Controls which target is built. Passed as `./waf ${VEHICLE}` to build the SITL. Also set as the value of the
`AP_VEHICLE` environment variable. By default this is set to `copter`

### `BRANCH`

Set the branch of ArduPilot to checkout prior to build. By default this is set to `ArduCopter-stable`
