# Creating your own controller

This documentation describes creating your own controller which is compatible with the Starling ecosystem. 

TODO: Complete this docs page

[TOC]

## Overview

## Implementation


### Launch Files

The controller itself can be run using `ros2 run`, but if you might want to run multiple nodes together, or provide some more complex functionality, it is recommended that ros2 nodes be run using launch files. ROS2 launch files can be written as xml or python programmes. Here we will explain using xml which imo is clearer, but the python is much more flexible. 

#### Basic launch file:

Here is an example of a launch file which should be saved as `controller.launch.xml` in a `launch` folder in the root of the repository: 
```xml
<launch>
    <arg name="vehicle_namespace" default="$(env VEHICLE_NAMESPACE vehicle_$(env VEHICLE_MAVLINK_SYSID))" />
    <arg name="demonstration" default="true"/>
    <group>
        <push-ros-namespace namespace="$(var vehicle_namespace)"/>
        <!-- <node name="primitive_4pl_controller" pkg="primitive_4pl" exec="controller" output="screen" respawn="true"> -->
        <node name="primitive_4pl_controller" pkg="primitive_4pl" exec="controller" output="screen">
            <param name="use_sim_time" value="$(env USE_SIMULATED_TIME false)"/>
            <param name="frame_id" value="map"/>
            <param name="setpoint_frame_id" value="$(var vehicle_namespace)/setpoint"/>
            <param name="vehicle_frame_id" value="$(var vehicle_namespace)/body"/>
        </node>

        <node name="primitive_4pl_controller" pkg="primitive_4pl" exec="demonstration" output="screen" if="$(var demonstration)"/>
    </group>

</launch>
```

This launch file takes two arguments which can be set by the user. Importantly the `vehicle_namespace` is set by the environment varible `VEHICLE_NAMESPACE` which is populated by an initialsation script in the controller base. 

Then to ensure that the controller is an `onboard` controller, so we have one controller running for one vehicle, we create a group and set the namespace to vehicle namespace. As long as the topic names used inside of the controller are not absolute (i.e. no leading `/` - `mytopic` vs `/mytopic`), the nodes topics will get mapped to `vehicle_1/mytopic`. 

Two nodes are started inside this node group. The first one requires a number of parameters which can need to use the vehicle name. The second one doesnt need any. 

#### Enabling running In simulation
The base controller has a USE_SIMULATED_TIME environment variable. Any node in the controller launchfile should include the following line if the controller requires use of time:

```xml
<node pkg="mypkg" exec="controller" ...>
    <param name="use_sim_time" value="$(env USE_SIMULATED_TIME false)"/>
   ...
</node>
```

So when you want to run the controller in simulation, you can simply set the `USE_SIMULATED_TIME` to true. For example `docker run --it --rm -e USE_SIMULATED_TIME mycontroller`. 

### Build and Bake Files

A bake file is a configuration file used to specify how to build a particular dockerfile. We use it in particular because it allows us to build cross-platfrom executables in the case we want to run our containers on a drone. A drone running a raspberry pi runs the `arm64` architecture, vs your own machine which most likely runs the `amd64` architecture. 

### Basic Bake file:
TODO: Explain what this means

```
variable "BAKE_VERSION" {
    default = "latest"
}

variable "BAKE_REGISTRY" {
    default = ""
}

variable "BAKE_RELEASENAME" {
    default = ""
}

variable "BAKE_CACHEFROM_REGISTRY" {
    default = ""
}

variable "BAKE_CACHETO_REGISTRY" {
    default = ""
}

variable "BAKE_CACHEFROM_NAME" {
    default = ""
}

variable "BAKE_CACHETO_NAME" {
    default = ""
}

/*
 * Groups for target ordering
 */
group "stage1" {
    targets = ["4pl"]
}

// This target depends on starling-controller-base
target "4pl-controller" {
    context = "."
    args = {
        "VERSION": "${BAKE_VERSION}",
        "REGISTRY": "${BAKE_REGISTRY}"
        }
    tags = [
        "${BAKE_REGISTRY}uobflightlabstarling/4pl-controller:${BAKE_VERSION}",
        notequal("",BAKE_RELEASENAME) ? "${BAKE_REGISTRY}uobflightlabstarling/4pl-controller:${BAKE_RELEASENAME}": "",
        ]
    platforms = ["linux/amd64", "linux/arm64"]
    cache-to = [ notequal("",BAKE_CACHETO_NAME) ? "${BAKE_CACHETO_REGISTRY}uobflightlabstarling/4pl-controller:${BAKE_CACHETO_NAME}" : "" ]
    cache-from = [ notequal("",BAKE_CACHEFROM_NAME) ? "${BAKE_CACHEFROM_REGISTRY}uobflightlabstarling/4pl-controller:${BAKE_CACHEFROM_NAME}" : "" ]
}
```

### Make Files

It is recommended that make (or nmake for windows) be used as a tool for building your controller. Make is a useful tool as it allows the running of any number of specific commands (similar to a bash file but with arguments dealt with for you). It is recommended that the makefile look something like the following, and be put in the root of the project folder:

```make
MAKEFILE_DIR:=$(shell dirname $(realpath $(firstword $(MAKEFILE_LIST))))
BAKE_SCRIPT:=$(MAKEFILE_DIR)/docker-bake.hcl
BUILDX_HOST_PLATFORM:=$(shell docker buildx inspect default | sed -nE 's/^Platforms: ([^,]*),.*$$/\1/p')
BAKE:=docker buildx bake --builder default --load --set *.platform=$(BUILDX_HOST_PLATFORM) -f $(BAKE_SCRIPT)

CONTROLLER_NAME=controller
NETWORK?=4pl-ros2-controller_default
ENV?=
BUILD_ARGS?=

all: build

build:
	$(BAKE) $(BUILD_ARGS) $(CONTROLLER)

# This mybuilder needs the following lines to be run:
# docker run --rm --privileged multiarch/qemu-user-static --reset -p yes
# docker buildx create --name mybuilder
# docker buildx use mybuilder
# docker buildx inspect --bootstrap
local-build-push:
	docker buildx bake --builder mybuilder -f $(BAKE_SCRIPT) --push $(CONTROLLER_NAME)

run: build
	docker run -it --rm --net=$(NETWORK) $(ENV) -e USE_SIMULATED_TIME=true $(CONTROLLER_NAME):latest

run_bash: build
	docker run -it --rm --net=$(NETWORK) -e USE_SIMULATED_TIME=true $(CONTROLLER_NAME):latest bash

.PHONY: all build local-build-push run run_bash
```
Breaking down the make file, we have a number of commands. 

```make
MAKEFILE_DIR:=$(shell dirname $(realpath $(firstword $(MAKEFILE_LIST))))
BAKE_SCRIPT:=$(MAKEFILE_DIR)/docker-bake.hcl
BUILDX_HOST_PLATFORM:=$(shell docker buildx inspect default | sed -nE 's/^Platforms: ([^,]*),.*$$/\1/p')
BAKE:=docker buildx bake --builder default --load --set *.platform=$(BUILDX_HOST_PLATFORM) -f $(BAKE_SCRIPT)
```
The first 4 lines define a number of useful variables including where the current directory is, the location of your bake script, your system's curent platform to build for and the final buildx bake command for building your controller

```make
CONTROLLER_NAME=controller
NETWORK?=controller_default
ENV?=
BUILD_ARGS?=
```
Then we define some useful variables to us, including the name of the controller and the local network (set to the default of foldername_default). Note that ENV and BUILD_ARGS have been set to a default of empty string on purporse. Then when running the file these can be specified e.g. `make ENV=-e HELLO=mynewvariable`. 

```make
all: build

build:
	$(BAKE) $(BUILD_ARGS) $(CONTROLLER)

# This mybuilder needs the following lines to be run:
# docker run --rm --privileged multiarch/qemu-user-static --reset -p yes
# docker buildx create --name mybuilder
# docker buildx use mybuilder
# docker buildx inspect --bootstrap
local-build-push:
	docker buildx bake --builder mybuilder -f $(BAKE_SCRIPT) --push $(CONTROLLER_NAME)

run: build
	docker run -it --rm --net=$(NETWORK) $(ENV) -e USE_SIMULATED_TIME=true $(CONTROLLER_NAME):latest

run_bash: build
	docker run -it --rm --net=$(NETWORK) -e USE_SIMULATED_TIME=true $(CONTROLLER_NAME):latest bash
```
Then we specify the commands. Here we have specified 5 commands: `all`, `build`, `local-build-push`,`run` and `run_bash`. The important one is `build` which can be run using `make build` which builds the container. A useful is `run` which will both build and run the newly built docker container. `run_bash` is the same as `run` except it puts you into the bash shell of the container instead of directly running. `local-build-push` is a helper which will help you push your container to dockerhub if you so need to. 

```make
.PHONY: all build local-build-push run run_bash
```
This final line is makefile syntax just in case you have any local files which are accidentally named exactly the same as one of the named commands.