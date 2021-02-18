# Project Starling

## Build & Run

There are two methods to run:

### 1. Container Method

With `make` installed, run `make` in the root folder to build the docker images
Use `make run` to start the gazebo, sitl and mavros containers

Go to http://127.0.0.1:8080 to (hopefully) see the simulator

Use this method for quick and easy local testing on a single machine.

### 2. Kubernetes Deployment Method

Use this method for simualting real drones or using real drones.

All container images required have already been prebuilt and are either on [uobflightlabstarling docker hub](https://hub.docker.com/orgs/uobflightlabstarling/repositories) or somewhere else.

In this file directory run `./run_k3s.sh` this will install k3s (lightweight kubernetes) and run the gazebo/ px4-sitl/ mavros example, equivalent to `make run`. This will (automatically) open up gazebo web on http://10.43.226.5:8080/ and the local k3s dashboard.

Opening a ground control station (gcs) program such as QGroundControl and creating a comms link to `udp://localhost:14553` and `udp://10.42.0.1:14553` will allow local monitoring.

A drone running the specified 64 bit debian/ubuntu iso can be added to the cluster. Identifying the ip address of the pi, the root enabled (possibly password disabled) username. Then from this directory run (you wll need to neter local machine password to access the master node token)
```bash
./deployment/start_k3s_agent.sh <remote username> <remote ip address> <node name>
```
A mavros node can be placed on the drone by then applying the following configuration:
```bash
sudo k3s kubectl apply -f deployment/k8.mavros.arm64.yaml
```
See [the following README.md for further details](deployment/README.md)
## Container Details

The root `Makefile` delegates to submakes to build various images:
 - `starling-sim-base-core` containing Gazebo, ROS2 and gzweb
 - `starling-sim-base-px4` built on top of `-core`, containing the PX4 Gazebo
    plugins
 - `starling-sim-clover2` built on top of `-px4`, containing the Clover model
    and textures
 - `starling-sim-px4-sitl` containing just the PX4 SITL binary

A `docker-compose` file sets up three containers; one for the Gazebo server,
one for the PX4 SITL, and one for mavros.

The Gazebo server runs in the `starling-sim-clover2` image with port 8080
published to allow web connection. The default command for this image is a
`ros2 launch` for `clover2.launch.xml`. This includes the lower layer launch
script which starts Gazebo and gzweb. Additionally, it runs a `spawn_entity`
script to spawn an instance of the Clover model in Gazebo. This model comes
from the `/robot_description` topic which is published by ROS2's
`robot_state_publisher`.

### Scaling

Scaling this setup involves spawning additional models on the Gazebo server and
running additional SITL and MAVROS instances. The `px4-sitl` image provides an
environment variable: `PX4_INSTANCE`. By default this has a value of zero.
Incrementing this will also increment the port numbers for the simulator host
and the mavlink outputs. This means the model needs to be spawned with this
in mind. The `PX4_SITL_PORT` variable for the `sim-clover2` image provides the
hook to adjust the model spawning. The `fcu_url` argument to the launch file
for the `mavros` image provides the adjustment for the MAVROS side.

Example of how scaling might work is in `docker-compose.multiple.yml` but this
hasn't been tested.

# Ideas

Ideally, baking the model into the image would not be required. Unfortunately,
gzweb needs model-specific assets before it can display a model. Custom plugins
may also be needed by the Gazebo server before a model can be loaded. Volumes
might provide a neater, if less user friendly way to do this.

## `ros.env.d`
Adding a folder to a `/ros.env.d` could provide an extendable way to add to the
ros environment. e.g. including additional model/plugin paths for Gazebo.