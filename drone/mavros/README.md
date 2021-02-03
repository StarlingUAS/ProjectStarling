# Mavros /ROS12 Bridge Container

To run:
```bash
make build # Builds docker container
make run # Runs docker container
```

Running the container will run 2 ros nodes in parallel:
1. A Mavros node listening on fcu_url, targeting drone with sysid tgt_system
   - `roslaunch mavros apm.launch fcu_url:=${MAVROS_FCU_URL} tgt_system:=${MAVROS_TGT_SYSTEM}`
2. A ros1 bridge node with option to forward all mavros topics to ros2
   - `ros2 run ros1_bridge dynamic_bridge --bridge-all-1to2-topics`

# Contents
There are two docker files:

- `Dockerfile.mavros` is the base container which is based on Ubuntu 20.04. It runs ROS2 Foxy, ROS1 Noetic and MAVROS via the ROS1_bridge.
- `mavros_setup.sh` is sourced at the end of the Dockerfile if any environment variables need modification.

## Configuration

We have identified that mavros is a base requirement for any drone running onboard compute in order to communicate with both the autopilot and GCS. 

The mavros container exposes an environment variable `MAVROS_FCU_URL` which is passed to mavros's `fcu_url` option for configuring the mavlink connection to mavros. The default value is:
> `MAVROS_FCU_URL=udp://127.0.0.1:14550@14555`

It also exposes environment variabel `MAVROS_TGT_SYSTEM` which is the `SYSID` of the ardupilot instance that mavros wants to connect to. This is usually an integer value. However, an IP address can also be passed in and `mavros_setup.sh` will extract the last numver of the IPv4 address as the value. Default is 1

To modify this at runtime, pass the `-e MAVROS_FCU_URL=<conenction string>` to `docker run` like so
> `docker run -e MAVROS_FCU_URL=<conenction string> -e MAVROS_TGT_SYSTEM=<#n>...`

See Mavros for possible connection values
