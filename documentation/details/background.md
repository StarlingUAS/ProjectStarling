# Background

This system has been tested on **linux ubuntu 20.04**. It should be compatible with any linux distribution compatible with Docker. It may work on windows and MacOS, however it is untested and there may be networking issues [See Link](https://github.com/arthurrichards77/ardupilot_sitl_docker#mission-planner-on-windows).

This system also assumes the use of ROS2 Foxy for communication between nodes, MavLink for autopilot communication with Mavros used as the bridge. PX4 is currently the main tested firmware, although ardupilot is also on the list. 

This system primarily uses containerisation technology ([Docker](https://www.docker.com/)) to encapsulate the dependencies of core systems. This means that running containered applications do not require any installation of extra dependencies apart from the Docker runtime. Full Install instructions are here: [https://docs.docker.com/engine/install/ubuntu/](https://docs.docker.com/engine/install/ubuntu/). The core container images (i.e. container executables) of this system are all hosted in the uobflightlabstarling docker hub repository: [https://hub.docker.com/orgs/uobflightlabstarling/repositories](https://hub.docker.com/orgs/uobflightlabstarling/repositories)

In our system each Docker Container can contain one or more ROS2 nodes. For example the `uobflightlabstarling / starling-mavros` container contains Mavros and its dependencies and require no modification by most users. Each docker container can be run by using `Docker run...`. Multiple containers can be run simultaneously using the `docker-compose` tool and specifying a yaml configuration file. 

When real drone hardware is added to the system a container deployment system is required, as docker (and docker-compose) only runs on one physical machine. [Kubernetes](https://kubernetes.io/) is what is known as a container orchestration platform controlling the deployment of groups of containers (known as pods) onto physical machines. The system can also be run within kubernetes locally on a single machine to test deployment before running on real hardware.

Note that most of this is for the users information, we have wrapped up most of this functionality into Makefiles for ease of use. For more information about specifics, see the wiki page (TODO). 