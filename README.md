# Project Starling

The purpose of Starling is to update the control systems within the Bristol Robotics Laboratory Flight Arena in order to reduce the barrier to entry for those who wish to experiment with, and fly real drones.

This project takes recent advancements in cloud computing, namely containerisation (docker) and container orchestration (kubernetes) as the core framework to manage networking and drone deployment, whether virtual (using SITL) or on real drones running companion computers (e.g. raspberry pi).

This systems provides a number of key features.

- Primary support for ROS2 and MavLink
- Built in simulation stack based on Gazebo
- Quick transition to flying controllers on real drones.

## Prerequisites

This system has been tested on **linux ubuntu 20.04**. It should be compatible with any linux distribution compatible with Docker. It may work on windows and MacOS, however it is untested and there may be networking issues [See Link](https://github.com/arthurrichards77/ardupilot_sitl_docker#mission-planner-on-windows).

This system also assumes the use of ROS2 Foxy for communication between nodes, MavLink for autopilot communication with Mavros used as the bridge. PX4 is currently the main tested firmware, although ardupilot is also on the list. 

This system primarily uses containerisation technology ([Docker](https://www.docker.com/)) to encapsulate the dependencies of core systems. This means that running containered applications do not require any installation of extra dependencies apart from the Docker runtime. Full Install instructions are here: [https://docs.docker.com/engine/install/ubuntu/](https://docs.docker.com/engine/install/ubuntu/). The core container images (i.e. container executables) of this system are all hosted in the uobflightlabstarling docker hub repository: [https://hub.docker.com/orgs/uobflightlabstarling/repositories](https://hub.docker.com/orgs/uobflightlabstarling/repositories)

In our system each Docker Container can contain one or more ROS2 nodes. For example the `uobflightlabstarling / starling-mavros` container contains Mavros and its dependencies and require no modification by most users. Each docker container can be run by using `Docker run...`. Multiple containers can be run simultaneously using the `docker-compose` tool and specifying a yaml configuration file. 

When real drone hardware is added to the system a container deployment system is required, as docker (and docker-compose) only runs on one physical machine. [Kubernetes](https://kubernetes.io/) is what is known as a container orchestration platform controlling the deployment of groups of containers (known as pods) onto physical machines. The system can also be run within kubernetes locally on a single machine to test deployment before running on real hardware.

Note that most of this is for the users information, we have wrapped up most of this functionality into Makefiles for ease of use. For more information about specifics, see the wiki page (TODO). 

## Running Starling Examples

The only dependency is that you have Docker installed [see prerequists](#Prerequisites). All container images required for the examples have already been prebuilt and are  on [uobflightlabstarling docker hub](https://hub.docker.com/orgs/uobflightlabstarling/repositories).

### 1. Docker Container

Use this method for quick and easy local testing on a single machine.

In the root directory, simply execute `make run` or `docker-compose up` in a terminal. This will start (1) Gazebo running 1 Iris quadcopter (2) A PX4-SITL instance (3) A Mavros node connected to the SITL instance (4) A simple UI with a go and estop button. 

> Note this might take a while to download the images off of hub.docker.com 

- Go to http://localhost:8080 in a browser to (hopefully) see the gazebo simulator
- Go to http://localhost:3000/html/main.html in a browser to see the starling user interface

If a mavros or sitl instance is running, there will be a GCS link on `udp://localhost:14553` (hopefully). This means that you can run a GCS such as QGroundControl, create a comms link to `localhost:14553` on UDP and you should be able to control any sitl drone through a standard mavlink interface. This is a quick an easy way to spin up a SITL instance.

An example offboard ROS2 controller can then be conncted to SITL by running the following in a separate terminal:
```bash
docker run -it --rm --network projectstarling_default uobflightlabstarling/example_controller_python
```
> This will download and run the `example_controller_python` image from `uobflightlabstarling` on docker hub. `-it` opens an interactive terminal. `--rm` removes the container when completed. `--network` attaches the container to the default network created by `make run` or `docker-compose`. The default network name is `<foldername>_default`.

When run, the example will confirm in the terminal that it has connected and that it is waiting for mission start. To start the mission, press the green go button in the [starling user interface](http://localhost:3000/html/main.html) which will send a message over `/mission_start` topic. A confirmation message should appear in the terminal, and the drone will arm (drone propellors spinning up) and takeoff. It will fly in circles for 10 seconds before landing and disarming. 

### 2. Kubernetes Deployment
Use this method for simualting real drone network architecture or using real drones.
#### Local Testing

In this file directory run `./run_k3s.sh` this will install k3s (lightweight kubernetes) and run the gazebo, px4-sitl, mavros and the go/stop ui containers example, equivalent to the docker command. This will (automatically) open up 
- Gazebo web on http://localhost:8080/, 
- GO/ESTOP UI on http://localhost:3000/html/main.html 
- Kubernetes dashboard on http://localhost:31771. To log in, you will need the login Token which is displayed by the `run_k3s` script. The token should also be automatically placed onto your clipboard for pasting. 

> Note: Dashboard may complain of self-signed certificates depending on browser. On Firefox click Advanced -> Continue. 

Opening a ground control station (gcs) program such as QGroundControl and creating a comms link to `udp://localhost:14553` and `udp://10.42.0.1:14553` will allow local monitoring.

The `example_controller_python` can then be run by running:
```bash
kubectl apply -f controllers/example_controller_python/k8.example_controller_python.amd64.yaml
```
Waiting a minute for download/ startup, pressing the Mission Start on the UI should get the Gazebo Drone to fly!

You can see the logs from the container by going to the dashboard (Pods -> s-example-controller-python-<randomstring> -> View Logs (top right icon on the blue ribbon))). 
#### Adding A drone to the cluster
A drone running the specified 64 bit debian/ubuntu iso can be added to the cluster. Identifying the ip address of the pi, the root enabled (possibly password disabled) username. Then from this directory run (you wll need to neter local machine password to access the master node token)
```bash
./deployment/start_k3s_agent.sh <remote username> <remote ip address> <node name>
```
A mavros node can be placed on the drone by then applying the following configuration:
```bash
sudo k3s kubectl apply -f deployment/k8.mavros.arm64.yaml
```
<!-- See [the following README.md for further details](/deployment/README.md) -->

## Implementing a Controller
### Modifying the example controller
In the [controllers](controllers) folder there is an example_controller_python which you should have seen in action in the example above. The ROS2 package is in [example_controller_python](controllers/example_controller_python/example_controller_python). Any edits made to the ROS2 package can be built by running `make` in the controllers directory. This will use colcon build to build the node and output a local image named `example_controller_python`. This local image can be run as follows:
```bash
docker run -it --name example_controller --rm --network projectstarling_default example_controller_python
```
> Note that because `uobflightlabstarling` is not referenced, it will look locally for a docker image. `--name` gives this instance a name which we can refer to.

Each container essentially runs its own operating system (see wiki for more details). Just as you could ssh into another machine, you can also inspect a running container:
```bash
docker exec -it example_controller bash
```
> Where `example_controller` is the name we gave the running instance. We essentially tell the container to execute `bash` for us to get a command line

Inside you can `source install/setup.bash` and run ROS2 commands like normal. 

### Creating your own from scratch

Of course you can create your own controller from scratch. Inside your controller repository, the following is required
1. Your ROS2 package folder (what would usually go inside the `dev_ws/src` directory)
2. A Dockerfile (named `Dockerfile`) which is dervied `FROM uobflightlabstarling/starling-controller-base`, use the [example Dockerfile](controllers/example_controller_python/Dockerfile) as a template. 

Your Dockerfile can be built by running the following in the directory with the Dockerfile.
```
docker build -t <name of your controller> .
```

Your container can then be run as above.

## Core Starling Containers

### Building

With `make` installed, run `make` in the root folder to build the docker images
Use `make run` to start the gazebo, sitl and mavros containers

### Container Details

The root `Makefile` delegates to submakes to build various images:
 - `starling-sim-base-core` containing Gazebo, ROS2 and gzweb
 - `starling-sim-base-px4` built on top of `-core`, containing the PX4 Gazebo plugins
 - `starling-sim-clover2` built on top of `-px4`, containing the Clover model and textures
 - `starling-sim-iris` built on top of `-px4`, containing the Iris model and textures
 - `starling-sim-px4-sitl` containing just the PX4 SITL binary
 - `starling-mavros` built on top of `ros:foxy` and runs ROS1 Mavros with the ROS1/2 Bridge to expose ROS2 mavros topics.
 - `starling-ui` built on top of `ros:foxy` and constructs the simple go/stop ui.

A `docker-compose` file sets up three containers; one for the Gazebo server,
one for the PX4 SITL, and one for mavros.

The Gazebo server runs in the `starling-sim-clover2` image with port 8080
published to allow web connection. The default command for this image is a
`ros2 launch` for `clover2.launch.xml`. This includes the lower layer launch
script which starts Gazebo and gzweb. Additionally, it runs a `spawn_entity`
script to spawn an instance of the Clover model in Gazebo. This model comes
from the `/robot_description` topic which is published by ROS2's
`robot_state_publisher`.

## Extra Notes
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

## Additional Ideas

Ideally, baking the model into the image would not be required. Unfortunately,
gzweb needs model-specific assets before it can display a model. Custom plugins
may also be needed by the Gazebo server before a model can be loaded. Volumes
might provide a neater, if less user friendly way to do this.

### `ros.env.d`
Adding a folder to a `/ros.env.d` could provide an extendable way to add to the
ros environment. e.g. including additional model/plugin paths for Gazebo.
