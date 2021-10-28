# Project Starling Ecosystem

ProjectStarling can be considered an umbrella term for a large ecosystem of projects which are connected by the plug and play modularity of the systems. This page lists some of the key github projects and containers which are used throughout this project. In the future this page can hopefully also link to extension modules which have been developed by users.

## Contents
[TOC]

## Core Starling Projects and Containers

### Project Starling

link - [https://github.com/UoBFlightLab/ProjectStarling](https://github.com/UoBFlightLab/ProjectStarling)

This repository contains the core elements of the system. It contains the source for the following docker containers:

* [uobflightlabstarling/starling-mavros](https://hub.docker.com/repository/docker/uobflightlabstarling/starling-mavros) - Core mavros container
* [uobflightlabstarling/starling-ui](https://hub.docker.com/repository/docker/uobflightlabstarling/starling-ui) - Basic UI (somewhat deprecated)
* [uobflightlabstarling/starling-controller-base](https://hub.docker.com/repository/docker/uobflightlabstarling/starling-controller-base) - Controller Base Container
* [uobflightlabstarling/starling-vicon](https://hub.docker.com/repository/docker/uobflightlabstarling/starling-vicon) - Container for vicon

In addition it also contains the source for the following containers for simulation

* [uobflightlabstarling/starling-sim-base-core](https://hub.docker.com/repository/docker/uobflightlabstarling/starling-sim-base-core) - Base Gazebo simulator

* [uobflightlabstarling/starling-sim-base-px4](https://hub.docker.com/repository/docker/uobflightlabstarling/starling-sim-base-px4) - Base container with px4
* [uobflightlabstarling/starling-sim-px4-sitl](https://hub.docker.com/repository/docker/uobflightlabstarling/starling-sim-px4-sitl) - Gazebo container with px4 sitl installed
* [uobflightlabstarling/starling-sim-iris](https://hub.docker.com/repository/docker/uobflightlabstarling/starling-sim-iris) - Base gazebo container with px4 sitl installed and spwawns the iris quadcopter model

* [uobflightlabstarling/starling-sim-ardupilot-copter](https://hub.docker.com/repository/docker/uobflightlabstarling/starling-sim-ardupilot-copter) - Base Container with ArduCopter
* [uobflightlabstarling/starling-sim-ardupilot-gazebo](https://hub.docker.com/repository/docker/uobflightlabstarling/starling-sim-ardupilot-gazebo) - Gazebo Simulator for use with Ardupilot
* [uobflightlabstarling/starling-sim-iris-ap](https://hub.docker.com/repository/docker/uobflightlabstarling/starling-sim-iris-ap) - Base gazebo container with ardupulot sitl installed and spawns the iris quadcopter model with camera. 

It also contains some example usage:

* [uobflightlabstarling/example_controller_python](https://hub.docker.com/repository/docker/uobflightlabstarling/example_controller_python)

### Murmuration

link - [https://github.com/StarlingUAS/Murmuration](https://github.com/StarlingUAS/Murmuration)

This repository contains all of the docker-compose and kubernetes deployment files. These deployment files rely on a number of the containers in this file.

### Starling Simple Offboard

link - [https://github.com/mhl787156/starling_simple_offboard](https://github.com/mhl787156/starling_simple_offboard)

This repository contains a basic PX4 simple offboard controller, abstracting away mavros.

* [uobflightlabstarling/starling-simple-offboard](https://hub.docker.com/repository/docker/uobflightlabstarling/starling-simple-offboard)

### Starling Allocator

link - [https://github.com/mhl787156/starling_allocator](https://github.com/mhl787156/starling_allocator)

This repository contains a trajectory allocator project allocating trajectories to visible mavros vehicles on the network (running starling simple offboard)

* [uobflightlabstarling/starling-allocator](https://hub.docker.com/repository/docker/uobflightlabstarling/starling-allocator)

### Starling Dash UI

link - [https://github.com/mhl787156/starling_ui_dashly](https://github.com/mhl787156/starling_ui_dashly)

This respository contains a more extensible replacement GUI for thie basic starling ui. It can be used to upload trajectories for use with the allocator and simple offboard.

* [uobflightlabstarling/starling-ui-dashly](https://hub.docker.com/repository/docker/mickeyli789/starling-ui-dashly)

## Specific and Applied Projects

### Coex Clover (clover_ros2_pkgs)

link - [https://github.com/UoBFlightLab/clover_ros2_pkgs](https://github.com/UoBFlightLab/clover_ros2_pkgs)

This repository contains libraries and dockerfile for building the software layer which controls hardware on the Clover drone

* [uobflightlabstarling/starling-clover](https://hub.docker.com/repository/docker/uobflightlabstarling/starling-clover)