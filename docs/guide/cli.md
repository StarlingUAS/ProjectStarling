# Starling CLI

This guide/tutorial provides a top to bottom set of instructions on how to use the Starling Command Line Interface present within the [Murmuration repostiroy](https://github.com/StarlingUAS/Murmuration).

This guide will be useful to those who have completed the first introduction and wish to start using starling more actively, especially with multiple vehicles.

[TOC]

## Background
The starling cli is comprised of a number of bash and python scripts to help streamline the usage of common aspects of the Starling vehicle controller simulation stack.

It is envisioned as aiding the process of following the Starling application development workflow:

1. Developing and testing the application locally in containers on a single simulated drone
2. Developing and testing the application locally within a simulated network archietcture of the flight arena using single or multiple simulated drones
3. Developing and testing the application on real vehicles within the flight arena.

The only requirement of the starling cli is a terminal running a shell and git. However please make sure you are running on a suitable computer as some of the applications are quite resource intensive.

## Installation

There are no installation steps apart from cloneing this github repository into your local workspace.
```console
git clone https://github.com/StarlingUAS/Murmuration.git
```

In the root of the repository, there is the core cli script named `starling`. `starling` includes a further installation script to help install further requirements. This installation script will need to be run using root. See the following guide on which arguments you should give.

> If running within Murmuration, swap `starling` for `./starling`. However for convenience, you can put `starling` onto your path. This can be done by adding `export PATH=<Path to murmuration>:$PATH` into `~/.bashrc` and `source ~/.bashrc` , or running the same command locally in the terminal. Then you can use the below commands verbatim.

### Only local development (step 1)
To install the bare minimum requirements of starling, run the following.
```console
sudo starling install
```
This will

- Update the system
- Install python3, pip, curl, udmap and a python utility called click
- Add the Mumuration directory to your path so you can run starling from anywhere
- Install docker

### Multi-Vehicle Local testing (step 2)

For this we utilise kubernetes in docker, a.k.a *kind* [link](https://kind.sigs.k8s.io/docs/user/quick-start/):
```console
sudo starling install kind
```

This is an application which will allow us to test out kubernetes deployments within a docker container without needing to install kubernetes elements locally. It also allows windows and mac users to develop starling applications as well.

### Multi-Vehicle Multi-Computer Setup

The final step allows you to setup the setup used in the flight arena using multiple physical machines. For this, we utilise k3s.

> This is not recommended for most users. Most should be able to use kind for testing, to then deploy straight in the flight arena.

```console
sudo starling install k3s
```

## Docker-Compose Usage

The `starling` utility is made up of a number of useful scripts which can be used together to run the simulation and your own controllers. Using `starling` with `docker-compose` is very simple and essentially a drop in replacement.

Once `starling` has installed, a docker compose configuration yaml file can be run with the following command:
```bash
starling deploy -f <docker compose file>
# e.g.
starling deploy -f docker-compose.px4.yaml
# or if the system needs to build a local container
starling deploy -f docker-compose.px4.yaml --build
```

The container can then be stopped by pressing `CTRL+C`.

If other docker-compose commands are required, you can just use the `docker-compose` cli.

If you want to run a container standalone, please refer to normal usage of `docker run ....`.

## Kind Usage

The `starling` utility is made up of a number of useful scripts which can be used together to run the simulation and your own controllers

### Starting or stopping the kind cluster

Running the following will give you a single drone cluster
```
starling start kind
```

If you are looking to experiment with a multi-drone system, you can specify the number using the `-n` option. For example for a 3 node (drone) cluster
```
starling start kind -n 3
```

To stop the cluster, simply run the following. Note that this will delete all changes within the kind cluster and maybe require re-setup.
```
starling stop kind
```

### Monitoring:


> For more details, see the [following tutorial](https://docs.starlinguas.dev/details/kubernetes-dashboard/) for an illustrated guide.

Once started, you can start the monitoring dashboard using the following command:

```
starling start dashboard
```

This will start up the [kubernetes dashboard](https://kubernetes.io/docs/tasks/access-application-cluster/web-ui-dashboard/). To access the dashboard, open up a browser and go to https://localhost:31771.

> Note the browser may not like the security, we promise it is safe to access! If you click 'advanced options' and 'continue to website' you should be able to access the website.

To log on, the above command should show something like the following:
```console
The Dashboard is available at https://localhost:31771
You will need the dashboard token, to access it.
Copy and paste the token from below
-----BEGIN DASHBOARD TOKEN-----
<LONG TOKEN>
-----END DASHBOARD TOKEN-----
Note: your browser may not like the self signed ssl certificate, ignore and continue for now
To get the token yourself run: kubectl -n kubernetes-dashboard describe secret admin-user-token
```
You can copy the `<LONG TOKEN>` and paste it into the dashboard token. You can also get the access token again by running:
```
starling utils get-dashboard-token
```

You can also monitor the system using
```bash
starling status
# or to continually watch
starling status --watch
```

And finally, you can inspect the system using the standard [`kubectl` commands](https://kubernetes.io/docs/reference/kubectl/cheatsheet/)

### Running the simulator

Once the simulator in kind has started, we can start the general simulator.

> **IMPORTANT**: The simulator can potentially be resource heavy to run. This is with respect to both CPU usage and RAM. Before going further, please ensure you have enough space on your primary drive (at least 30Gb to be safe, especially C drive on windows). This is especially true if running multiple vehicles. It is not recommended to run more than around 6.

First we should load or download the required simulation containers locally. This can be done using the follwoing command. We need to run the load command as we want to load the local container images into the kind container. This avoids the need for the kind containers to download the containers themselves at each runtime.

> This command can take as long as 30 minutes depending on internet connection. It goes through the deployment files and downloads a couple of large containers e.g. the gazebo and sitl containers.

> If the container doesn't already exist locally, it will attempt to download it from docker hub

```
starling simulator load
```

Once containers are downloaded and loaded into Kind, we can start the simulator containers using the following:

```bash
starling simulator start
# or to do both steps at the same time:
starling simulator start --load
```

Once run, you can monitor the deployments on the kubernetes dashboard. In particular you can inspect the **worloads** -> **pods**. If they show green the systems should hopefully have started correctly.

> Again, see [this illustrated guide](https://docs.starlinguas.dev/details/kubernetes-dashboard/) to the dashboard

> Note, if the load was unsucessful, or had issues, some containers might take a while to download starling containers.

Once started, the simulator should be available to inspect from https://localhost:8080

At this point, you can run your own deployments or some of the examples.

If at any point, you want to restart the simulation, you can run the following. This will delete the starling simulator containers, and then start them again.
```bash
starling simulator restart
```

If you want to stop the simulation, simply run
```bash
starling simulator stop
```

### Deploying containers

The primary usage of starling is to test your local containers. To do so, please follow the following steps.

First, if it is a local container - a container image which exists on your local machine either through having pulled it from docker hub, or built locally - you will need to load that image into kind. Each time you rebuild or change the image, you will need to load it into kind. This can be achieved using the following command:

> Note: If the container doesnt exist locally it will automatically try and download it from docker hub

```bash
starling utils kind-load <container name>
# e.g.
starling utils kind-load uobflightlabstarling/starling-mavros:latest
```

Secondly, you will need to write a kubernetes deployment file known as a kubeconfig. A kubeconfig is a yaml file which specifies what sort of deployment you want, along with many options (where to run your controller etc.). For now, see the *deployment* folder and the repositories in StarlingUAS for examples. In particular we mention the existence of two types of deployment:

1. **Deployment**: This specfies the self-healing deployment of one *pod* (a.k.a a collection of containers) to a node.
2. **DaemonSet**: This specifies the self-healing automatic deployment of a pod to all nodes which match a given specification. Use this for a deployment to all vehicles.

Once a kubernetes configuration has been written, it can be deployed to kind.
```bash
starling deploy -k <path to kubeconfig file> start
```

> *Note* if you have added starling to path (through standard installation or otherwise), you can run this from any directory, in particular the directory of the project you wish to deploy. If you haven't, you may need to give a full absolute path.

> *Note* if kind is not active or installed, `starling deploy` will interpret commands for `docker-compose`.

To stop or restart a deployment you can use the following:
```bash
starling deploy -k <path to kubeconfig file> stop
starling deploy -k <path to kubeconfig file> restart
```

To restart a deployment with an update to the container, you just need to ensure you have loaded it in before you restart:
```bash
starling utils kind-load <container name>
starling deploy -k <path to kubeconfig file> restart
```

Again, use the dashboard to monitor the status of your deployment

# Docker-Compose Examples

The [`docker-compose`](docker-compose) contains a number of preset examples which spin up at minimum a simulator, a software autopilot, mavros and a ros webridge, but also example ui's and basic controllers.

Each example file has a *linux* and *windows* variant.
- The *linux* variant runs `network_mode=host` which allows for ROS2 traffic to be shared between the container and your local network. Therefore the traffic is visible to your machine's local (non-container) ROS2 foxy instance, allowing you to run bare-metal application such as rviz2 or your own controllers (i.e. you do not need to wrap your own controllers in a docker container). Any exposed ports are automatically exposed to `localhost`.
- The *windows* variant runs inside a docker-compose network named `<folder-which the-docker-compose-file-is-in>_default`, (e.g. running a px4 example will create a local network `px4_default`). This network is segregated from your local network traffic *except* for the exposed ports in the docker-compose file which are now accessible from `localhost` (Windows has no support of `net=host`). Any other ROS2 nodes will need to be wrapped in a docker container for running and run with `--network px4_default` or `--network ardupilot_default`. See the example controller repository for an example ROS2-in-docker setup.

## PX4 Examples

The [docker-compose/px4](docker-compose/px4) folder contains a number of px4 based examples. See the [README](https://github.com/StarlingUAS/Murmuration/blob/main/docker-compose/px4/README.md) for more details.

### Core System
To start the simulator, a px4 SITL, mavros and the web-bridge, use the following command:
```bash
# Pull and Run the docker-containers
starling deploy -f docker-compose/px4/docker-compose.core.linux.yml --pull
# or for windows
starling deploy -f docker-compose/px4/docker-compose.core.windows.yml --pull
```

### Simple Offboard with Trajectory Follower UI System
To start the core with a [simple-offboard controller](https://github.com/StarlingUAS/starling_simple_offboard), [simple-allocator](https://github.com/StarlingUAS/starling_allocator) and [Trajectory follower Web GUI](https://github.com/StarlingUAS/starling_ui_dashly), use the following command:

```bash
# Run the docker-containers
starling deploy -f docker-compose/px4/docker-compose.simple-offboard.linux.yml --pull
# or for windows
starling deploy -fdocker-compose/px4/docker-compose.simple-offboard.windows.yml --pull
```

## Ardupilot Examples

The [docker-compose/ardupilot](docker-compose/ardupilot) folder contains a number of px4 based examples. See the [README](https://github.com/StarlingUAS/Murmuration/blob/main/docker-compose/ardupilot/README.md) for more details.

### Core System
To start the gazebo simulator, an arducopter SITL, mavros and the web-bridge, use the following command:
```bash
# Run the docker-containers
starling deploy -f docker-compose/ardupilot/docker-compose.ap-gazebo.linux.yml --pull
# or for windows
starling deploy -f docker-compose/ardupilot/docker-compose.ap-gazebo.windows.yml --pull
```

## Running External Examples
An example offboard ROS2 controller can then be conncted to SITL by running the following in a separate terminal. For this we have to use `docker`'s built in command line interface.

```
# Download the latest container
docker pull uobflightlabstarling/example_controller_python

docker run -it --rm --network px4_default uobflightlabstarling/example_controller_python
# or for ardupilot
docker run -it --rm --network ardupilot_default uobflightlabstarling/example_controller_python
```

See [the docs](https://docs.starlinguas.dev/guide/single-drone-local-machine/#2-running-example-ros2-offboard-controller-node) for further details

# Kubernetes Examples

A folder of example deployments is provided in the repository. Currently there is only the one example, but hopefully more will follow! These can all be run using the `deploy` command with the name of the example (corresponds to name of the folder)
```bash
starling deploy example <example-name>
#e.g.
starling deploy example simple-offboard
```

1. **simple-offboard** - This example deploys 3 elements which together allow the running of simple waypoint following examples through a graphical user interface on one or more UAVs. Can be run using `starling deploy example simple-offboard` once the simulator has been initialised
    1. A daemonset is used to deploy the [simple offboard controller](https://github.com/StarlingUAS/starling_simple_offboard) to each vehicle. This provides a high level interface for controlling a vehicle
    2. An example [Python based UI using the dash library](https://github.com/StarlingUAS/starling_ui_dashly). This provides a graphical user interface to upload and fly trajectories.
    3. An [allocator module](https://github.com/StarlingUAS/starling_allocator) which takes a trajectory from the UI and distributes it to a particular vehicle to fly.
