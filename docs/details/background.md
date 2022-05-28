# Background

## Motivation

As robotic and autonomous systems proliferate into the wider world, there is a need to address the difficulties of system development and deployment at scale. There is evidence that industry is directly facing these challenges through the use of cloud computing, continuous integration and similar systems inspired from very successful and agile software development processes. This is made clear through offerings such as Amazon's [AWS Robomaker](https://aws.amazon.com/robomaker/), Google's cloud robotics [platforms](\url{https://googlecloudrobotics.github.io/core/) and so on.

However, there is a great lack of such systems in most academic settings. The result's oriented attitude of many labs often leads to each researcher building a bespoke solution in order to evaluate, validate or prove their goals. These bespoke solutions are often inflexible, not extensible, difficult to understand and, importantly, reuse, with any level of confidence. This becomes especially difficult when coupled with hardware, such as UAVs, where many operational details have been implicitly assumed or ignored for favour of getting the experiment running as quick as possible. In addition these solutions are often poorly structured and maintained  with little to no documentation meaning that it is difficult for researchers to build upon these systems. This is an exceptionally large hurdle to researchers who do not have strong software backgrounds, but wish to perform real world experiments which could improve the quality of research outputs.

This is not to say that it is impossible for a research system to be developed into a reusable platform. There are many examples of research systems being ubiquitous within a group or being released outside the lab. For instance, the [Robotarium at Georgia Tech](https://www.robotarium.gatech.edu/), the Multi-Robot Systems Group at the Czech Technical University with their [experimental system](https://github.com/ctu-mrs/mrs_uav_system), and the PX4 autopilot which began it's life as a collaboration between a number of labs at ETH Zurich. But what we see is that it takes a concerted effort and many years of coincidental work which provide incremental improvements to the system each time.

Inspired by the increasing adoption of paradigms from cloud and distributed computing, this work aims to provide a UAV experimentation system which:

1. Supports single or multiple drones with low (control) or high level (path planning) experimentation.
2. Supports the transition between simulation to indoor flight to outdoor flight.
3. Provides a simple and easy to use interface for researchers to implement experimentation without knowledge of hardware specifics.

## System Overview

Starling makes use of containerisation ([Docker](https://www.docker.com/)) and container orchestration technologies (Kubernetes) to enable consistent deployment environments regardless of underlying platform - whether simulated or real. Many previous systems also aim for a similar capability, but the use of containerisation in particular allows Starling to abstract away from hardware whilst minimising setup and configuration. This is key to increase the efficiency for the development and testing of UAV systems in our resource and time constrained research laboratory. Starling has been primarily been devised with the following two insights:

Firstly, a key concept of Starling is to treat individual vehicles as if they are individual nodes in a compute cluster. A compute cluster is traditionally a physical set of servers within a data centre, where each node is one hardware server with its own resources. Starling therefore considers a group of UAVs as a mobile compute cluster where each UAV is a 'flying computer'. With this viewpoint, actions such as software deployment, scaling up and down, self-healing and others can start to have a physical meaning within a UAV application.

Secondly, Starling embraces the modularity inherent in using both ROS and containerisation. ROS provides the framework for communication, and containers provide the portable foundation upon which it runs. In particular Starling provides a core set of containers which have been identified as being mandatory when running a vehicle. These include a pre-configured container which runs MAVROS, as well as defining environment variables and protocols to ensure that vehicles are unique and can communicate in a scalable manner with both its hardware and other vehicles without additional user configuration. With this modularity through containers, we can then exploit all of the recent advances in compute cluster deployment and control for use in a single and multi UAV testing environment.

## Technology Stack

This system has been tested on **linux ubuntu 20.04**. It should be compatible with any linux distribution compatible with Docker. It may work on windows and MacOS, however it is untested and there may be networking issues [See Link](https://github.com/arthurrichards77/ardupilot_sitl_docker#mission-planner-on-windows).

This system also assumes the use of ROS2 Foxy for communication between nodes, MavLink for autopilot communication with Mavros used as the bridge. PX4 is currently the main tested firmware, although ardupilot is also on the list.

The core container images (i.e. container executables) of this system are all hosted in the uobflightlabstarling docker hub repository: [https://hub.docker.com/orgs/uobflightlabstarling/repositories](https://hub.docker.com/orgs/uobflightlabstarling/repositories)

In our system each Docker Container can contain one or more ROS2 nodes. For example the `uobflightlabstarling / starling-mavros` container contains Mavros and its dependencies and require no modification by most users. Each docker container can be run by using `Docker run...`. Multiple containers can be run simultaneously using the `docker-compose` tool and specifying a yaml configuration file.
