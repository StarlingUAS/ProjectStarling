# `starling-mavros2` Container

This container is the core Starling container. It contains the primary interface between the flight controller (simulated or real) and the users application written in ROS2.

This container is mainly comprised of the MAVROS, MAVLINK/ROS2 ros node which provides a standard ROS2 interface to the user. This container performs an automated set of modifications to allow easy connection to SITL or the real vehicle. It also contains a set of general nodes which should be part of every user application.

## Contents

[TOC]

## Overview

The image is a wrapper around MAVROS, as such, most of the documentation for MAVROS is of use when working with this
image. The image has both `mavros` and `mavros-extras` installed so all plugins should be available.

The published topics are namespaced to allow for running multiple vehicles on the same ROS network. By default, this
namespace will be `/vehicle_N`, where `N` is the MAVLink system ID of the vehicle. MAVROS topics are published within
this namespace, *e.g.* `/vehicle_1/mavros/state`.

The image has been setup to automatically configure itself in some scenarios:

 - Running on a vehicle
 - Running in Kubernetes

There are also some additional general nodes running in the background. In particular:

  - Emergency Stop listener on the `/emergency_stop` topic of type `std_msgs/msg/Empty`. It is hard-wired to forcefully disarm the vehicle (stop motors) for both PX4 and Ardupilot in simulation and reality.
  - Ping Listener on the `/ping_source` and `/ping_sink` topics which are used in conjunction with the `ping-monitor` to measure vehicle round trip time.


## Launch Process

The initial process of starting the container goes through `ros_entrypoint.sh`. This file sources ROS2 and
runs a setup script `mavros_setup.sh` which configures the environment to tell MAVROS how to talk to the drone and where to publish its
topics. The setup script works in a couple of different ways depending on where the container is running.

- If the container is running on a drone, it expects to be able to find the `/etc/starling/vehicle.config` file. This filecontains some information that MAVROS needs to be able to communicate with the flight controller. An example `vehicle.config` file is included below.

If the container is started as part of a Kubernetes StatefulSet deployment, the setup script will start a MAVROS2 Container instance based on the set numerical given by Kubernetes.

To build this container clone the ProjectStarling repo and cd in system.

```bash
cd system
make mavros2 # Builds docker container
docker run -it --rm uobflightlabstarling/starling-mavros2:latest
```

Default action of launching the container (make run) will be to launch the `mavros.launch.xml`.

## Launch file

The launch file `mavros.launch.xml` runs the following
Running the container will run the following ros nodes in parallel:

This launch file starts the following:

1. Mavros2 ROS2 node
2. The EStop Listener
3. The Ping Responder (To measure Round Trip Time)


## Contents

The container is built out of `Dockerfile` which is the base container which is based on Ubuntu 20.04. It runs ROS2 Foxy and MAVROS 2 with extra nodes.

> The `buildfromsource.Dockerfile` is an example Dockerfile for building MAVROS2 from source which has been kept for reference.

The other files include:

- `mavros_setup.sh` is sourced at the end of the Dockerfile if any environment variables need modification.
- `mavros.launch.xml` is a ros2 launch file, and launches both mavros and the extra nodes. It has 4 main parameters:
     - `fcu_url` - Flight control url, point it at the physical autopilot, ardupilot sitl or px4 sitl, no default in launch file. Defailts to `MAVROS_FCU_URL="udp://127.0.0.1:14550@14555"` using launch.sh
     - `target_system` - target id to filter for from mavlink stream, defaults to 1, or MAVROS_TGT_SYSTEM using launch.sh
     - `system_id` - sysid of mavros node, defaults to 1
     - `firmware` - `px4` or `apm`, name of the mavros launch file. Defaults to `px4`.
- `config` folder contains the parameter files which `mavros.launch.xml` reads in to setup what mavros2 functionality is available and how it is configured. See below.
- `starling_mavros_extras` folder contains the source code for any extra ROS2 nodes we wish to run inside this core container.
- `fastrtps_profiles.xml` is a ROS2 configuration file which specifies that it should always use UDP for sending data.

## Configuration

We have identified that mavros is a base requirement for any drone running onboard compute in order to communicate with bot h the autopilot and GCS.

The mavros container exposes an environment variable `MAVROS_FCU_URL` which is passed to mavros's `fcu_url` option for configuring the mavlink connection to mavros. The default value is:
> `MAVROS_FCU_URL=udp://127.0.0.1:14550@14555`

It also exposes environment variabel `MAVROS_TGT_SYSTEM` which is the `SYSID` of the ardupilot instance that mavros wants to connect to. This is usually an integer value. However, an IP address can also be passed in and `mavros_setup.sh` will extract the last numver of the IPv4 address as the value. Default is 1

To modify this at runtime, pass the `-e MAVROS_FCU_URL=<conenction string>` to `docker run` like so
> `docker run -e MAVROS_FCU_URL=<conenction string> -e MAVROS_TGT_SYSTEM=<#n>...`

See Mavros for possible connection values

It to gets the
ordinal of its containing pod from the hostname. It will then use this ordinal to set up the ports and the system ID to
match those generated by a PX4 SITL instance set up with the same ordinal. If the setup script fails to get the ordinal
from the hostname, it will attempt to connect to a PX4 SITL with `PX4_INSTANCE=0`.

Once the setup script has run, the default behaviour is to launch the `mavros.launch.xml` file. This behaviour
should be usable in almost all cases.

The __ROS2__ `mavros.launch.xml` script defines a set of arguments to enable configuration of the MAVROS node.
These are ususally filled by the environment variables defined below. If the configurability provided here is
insufficient, the image can be run with a different command.

In addition there exists the configuration parameter files in `config` which are used by mavros to specify which mavros plugins are running, and their associated parameters. These configuration files are copied from `config` into the root directory of the container (`/`). See below for how to use custom configurations.

During runtime, the `mavros_setup.sh` produces a modified version of these files to ensure that vehicle namespacing is understood. These are suffixed with `_mod` and should not be modified.

## Configuration Options

There are many configuration options available for this image to allow it to be used flexibly across different
deployment scenarios. The most important of these are those relating to the vehicle and GCS connections, and MAVROS
configuration. A full summary is given in the table below.

Name                      | Default Value                  | Description
--------------------------|--------------------------------|------------
`MAVROS_FCU_CONN`         | "udp"                          | Protocol for autogenerated FCU URL
`MAVROS_FCU_IP`           | "127.0.0.1"                    | IP for autogenerated FCU_URL
`MAVROS_FCU_UDP_BASE`     | "14830"                        | Base port for autogenerated FCU_URL
`MAVROS_TGT_SYSTEM`       | "auto"                         | Target system ID, if set to a number, this will __override__ the automatic behaviour
`PX4_INSTANCE_BASE`       | 0                              | Base instance for autogenerated instance matching
`MAVROS_TGT_FIRMWARE`     | "px4"                          | Firmware profile used by MAVROS. Only other valid value currently is "apm"
`MAVROS_GCS_URL`          | "udp-pb://@:14550"             | MAVROS URL for ground control station connection
`MAVROS_FCU_URL`          | {unset}                        | MAVROS URL for FCU connection. Set to __override__ automatic behaviour
`VEHICLE_NAMESPACE`       | {unset}                        | Namespace for mavros topics. Set to __override__ default value of `vehicle_${TGT_SYSTEM}`
`MAVROS_PLUGINLISTS_PATH` | "/mavros_pluginlists_px4.yaml" | Path for MAVROS pluginlists file
`MAVROS_CONFIG_PATH`      | "/mavros_config_px4.yaml"      | Path for initial MAVROS configuration file
`MAVROS_MOD_CONFIG_PATH`  | "/mavros_config_mod.yaml"      | Path for modified MAVROS config to be written to

### Configuring the Connection

MAVROS is told to connect to a MAVLink source with a URL. This URL can be configured in a number of ways. The most
straightforward is to simply set the `MAVROS_FCU_URL` environment variable. This will override any other behaviour.

```sh
docker run -e MAVROS_FCU_URL=serial:///dev/ttyUSB0:115200 uobflightlabstarling/starling-mavros
```

When setting `MAVROS_FCU_URL` directly, note that a query string (_e.g._ `?ids=n,240`) will be added during launch.
You need to ensure that your input for `MAVROS_FCU_URL` supports this syntax. Of particular note is the need for a
trailing `/` in most formats, but not for the `serial://` format. Also note that the plain file format does not support
this. See [the MAVROS docs](https://github.com/mavlink/mavros/blob/master/mavros/README.md#connection-url) for more
information on URL formats.

`MAVROS_FCU_URL` is can also be set automatically by the container depending on its environment. If there is a
`vehicle.config` file mounted in the image, the value of `VEHICLE_FCU_URL` from that file will be used as the `fcu_url`.
This is especially useful when deploying the container onto physical vehicles. The same warning about trailing slashes
above goes for the value put in `vehicle.config`.

If there is no `vehicle.config` file, the image will configure itself based on the values of `MAVROS_FCU_CONN`,
`MAVROS_FCU_IP` and `MAVROS_FCU_UDP_BASE`. An additional parameter, `INSTANCE` is also used in the construction of the
URL. This is generated based on the container hostname and is intended for use in Kubernetes deployments to distinguish
multiple instances. `PX4_INSTANCE_BASE` can be used to offset the `fcu_url` will be constructed as below:
> `$MAVROS_FCU_CONN://$MAVROS_FCU_IP:$((MAVROS_FCU_UDP_BASE + INSTANCE))@/`

With all values at default, this ends up as:
> `udp://127.0.0.1:14830@/`

### Configuring the Target System

Another important configuration option is the target system ID. This controls the target system that MAVROS sends in
some messages. As for the `fcu_url`, the value can be explicitly overridden, this time using the `MAVROS_TGT_SYSTEM`
environment variable. Setting this will overrise all other values. If it is set to an invalid system ID, MAVROS will
be set to use a target ID of `1`.

If the environment variable is left at its default value of `auto`, a similar flow to the `fcu_url` occurs: if a
`vehicle.config` file exists, the value of `VEHICLE_MAVLINK_SYSID` from that file will be used. Otherwise, the value is
autogenerated based on the `INSTANCE` parameter derived from the container hostname. Note that the `INSTANCE` number is
0-indexed, while MAVLink system IDs start at `1`. Therefore, the system ID is set to one more than the `INSTANCE`.

### Configuring the MAVROS Configuration

Two sets of `config.yaml` and `pluginlists.yaml` files are installed in the root directory to provide alternatives for
PX4 and ArduPilot autopilots. These are named: `/mavros_config_px4.yaml` and `/mavros_pluginlists_px4.yaml` for the PX4
versions and `/mavros_config_ap.yaml` and `/mavros_pluginlists_ap.yaml` for the ArduPilot versions.

The easiest way to choose between the two is to set the `MAVROS_CONFIG_PATH` and `MAVROS_PLUGINLISTS_PATH` environment
variables. By default these point to the PX4 versions. To use the ArduPilot versions set both variables as below:

```sh
docker run -e MAVROS_CONFIG_PATH=/mavros_config_ap.yaml -e MAVROS_PLUGINLISTS_PATH=/mavros_pluginlists_ap.yaml ...
```

It is also possible to mount alternative configurations into the image and use the environment variables to configure
MAVROS with them.

TODO: Example of mounted configuration

## Example `vehicle.config`

Note that the extended form of the serial URL is required for MAVROS's target "query string" to work.

```bash
VEHICLE_FCU_URL=serial:///dev/px4fmu:115200
VEHICLE_FIRMWARE=px4
VEHICLE_MAVLINK_SYSID=23
VEHICLE_NAME=clover23
```

## Implementation Details

### Running on a vehicle

Ensure `/etc/starling/vehicle.config` is mounted. The container is then configured from the contents of that file.

### Running under Kubernetes StatefulSet

The container is configured based on the detected ordinal from the hostname.

`MAVROS_FCU_URL` is autogenerated as: `udp://127.0.0.1:$((14830 + ORDINAL))@`

`MAVROS_TGT_SYSTEM` will end up as `$((ORDINAL + 1))`

### Running isolated

Default values will be used, equivalent to the Kubernetes case with `ORDINAL=0`

### Using custom mavros configurations

By default we have only enabled a subset of all the mavros plugins which are available. This is so that we limit the data flow. If your application requires other plugins, this can really easily be done.

First make a local copy of the `mavros_pluginlists....yaml` file and modify it to what you wish. The `plugin_denylist` specifies a list of plugins which should not run. The `plugin_allowlist` speicifes a list of plugins which should be run. Make sure not to change the name of the parameters (`mavros`). Then either manually using bind mounting, or within the docker-compose file, you should mount this file at `/mavros_pluginlists_px4.yaml`. This overwrites the default file.

### Weird Namespacing for mavros pluginlists

In ROS2 a parameter file has an annoying specific format. The first entry should be the name of the node which the following ros2 parameters apply to. This must directly map to the node name.

We would like our node to be under `<vehicle_namepsace>/mavros/...`. But by default the topics all come out on `...` essentially root. Therefore in the `mavros.launch.xml` we set the mavros node to use namespace `<vehicle_namepsace>/mavros`. However in order for the param file to match, we need to match the root mavros node name which is `<vehicle_namepsace>/mavros/mavros`.

Therefore the top of `mavros_pluginlists_px4.yaml` must specify the node name `<vehicle_namepsace>/mavros/mavros`. The template pluginlists only specifies `mavros`, so the `mavros_setup.sh` dynamically renames the pluginlist yaml file with the correct namespace.

## Advanced Topics

### Adding additional MAVROS plugins

This should be possible by mounting a volume with your plugin into the container. Assuming ROS tools are able to find
it, MAVROS should load the plugin (if directed by the pluginlists file).