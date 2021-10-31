# `starling-sim-ardupilot-gazebo`

Based on [`starling-sim-base-core`](../sim-base-core)

This image contains the base Gazebo installation and adds the ArduPilot specific plugin.

## Environment Variables

Name                  | Default Value                | Description
----------------------|------------------------------|------------
`AP_SYSID`            | 1                            | MAVLink system ID to be used by the SITL. Can also be set to `"ordinal"` or `"ip"`
`AP_SYSID_BASE`       | 1                            | Base system ID for ordinal-based generation
`AP_SITL_ADDRESS`     | 127.0.0.1                    | IP address for Gazebo plugin to use to talk to ArduPilot instance
`AP_SITL_HOST`        | {null}                       | Hostname for Gazebo plugin to use to talk to ArduPilot instance. Set to __override__ IP address.
`VEHICLE_NAMESPACE`   | `vehicle_${AP_SYSID}`        | ROS2 Namespace in which to place vehicle sensor topics (e.g. gimbal_cmd)

### `AP_SYSID`

Sets the MAVLink system ID to be used by the SITL. This must either be a value between 1 and 255 inclusive or set to
`"ordinal"` or `"ip"`. Other values will cause the container to exit.

When set to `"ordinal`, the entrypoint script will attempt to retrieve a 0-indexed ordinal from the end of the
container's hostname. The hostnames should be of the form `${HOSTNAME}-${ORDINAL}`, *e.g.* `sitl-0`, `sitl-1`, `sitl-3`.
This behaviour supports deployment of this image as part of a Kubernetes StatefulSet.

When set to `"ip"`, the entrypoint script retrieves the last octet of the container's IP address and uses that as the
system ID. *e.g.* `172.18.0.4` will result in a system ID of `4`.

At present, the system ID is set by appending it to the parameter file. If a custom parameter file is supplied, it
should not contain the `SYSID_THISMAV` parameter.

### `AP_SYSID_BASE`

This variable only affects the container when using the ordinal-based generation outlined above. The value set here will
be added to the ordinal from the hostname to derive the MAVLink system ID. If the resultant value is not a valid system
ID, *i.e.* it is not between 1 and 255 inclusive, the container will exit.