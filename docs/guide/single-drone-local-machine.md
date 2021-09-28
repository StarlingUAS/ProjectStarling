# Single Drone Local Machine

Follow these instructions for quick and easy testing of controllers on a single drone on a single local machine. **Use in the following scenarios**:

1. Local development of single drone applications in simulation (gazebo)
2. Local development of offboard (drone control software on pc not on drone)
3. Local development of onboard (drone control software running on drone itself)
4. No requirement of imitating the real drone software/ communication architecture

**This is considered to be step 1 for the Starling development process.**
## Contents
[TOC]

## Starting the Drone and Simulator (Simple Version)

There are two basic example starting confgurations, this 'Simple' version starts the bare minimum with the expectation of it being used through mavlink. There is also a 'Full' version with instructions below this section which starts a system capable of flying higher level paths and trajectories submitted through a GUI or ROS.

First check that you have installed the single prerequisit of `docker`, see [Getting Started](../guide/getting-started). You may also need to install `docker-compose` via `sudo apt-get install docker-compose`.

In the root directory first download the most up-to-date versions of the system using `docker-compose pull`. (Periodically run this to ensure you are updated). Then in a terminal, execute:

```
docker-compose -f docker-compose-simple.yml up
```

> **Note**: If on Windows using with a native windows GCS such as Mission Planner, you may need to use `docker-compose -f docker-compose-simple.tcp.yml up`

This will start the following:

1. Gazebo simulation enviornment running 1 Iris quadcopter model
2. A SITL (Software In The Loop) instance running the PX4 autopilot.
3. A Mavros node connected to the SITL instance
4. A simple UI with a go and estop button.

> **Note:** this might take a while (can be up to 30 min or an hour depending on internet) on first run as downloads are required.

> **Note:** Use ctrl+c to stop the process when you have finished.

The User Interfaces are available in the following locations:

- Go to [`http://localhost:8080`](http://localhost:8080) in a browser to (hopefully) see the gazebo simulator.
- Go to [`http://localhost:3000`](http://localhost:3000) in a browser to see the starling user interface containing go/stop buttons.

> **Note:** All specified sites can be accessed from other machines by replacing `localhost` with your computer's IP address.

> **Note:** Sometimes it might take a bit of time for the UIs to become available, give it a minute and refresh the page. With Gazebo you may accidentally be too zoomed in, or the grid may not show up. Use the mouse wheel to zoom in and out. The grid can be toggled on the left hand pane.

## Starting the Drone and Simulator (Full Version)

This confugration starts a slightly more complex system which is capable of automatic higher level navigation and trajectory following. Ensure that the 'simple' version has been stopped before using this one.

Again, first check that you have installed the single prerequisit of `docker`, see [Getting Started](../guide/getting-started). You may also need to install `docker-compose` via `sudo apt-get install docker-compose`.

In the root directory first download the most up-to-date versions of the system using `docker-compose pull`. (Periodically run this to ensure you are updated). Then in a terminal, execute:

```
docker-compose up
```

> **Note**: If on Windows using with a native windows GCS such as Mission Planner, you may need to use `docker-compose -f docker-compose.tcp.yml up`

This will start the following:

1. Gazebo simulation enviornment running 1 Iris quadcopter model
2. A SITL (Software In The Loop) instance running the PX4 autopilot.
3. A Mavros node connected to the SITL instance
4. A Simple Offboard PX4 controller which provides higher level navigation actions
5. A UI with a go, abort and estop button, as well as a trajectory uploading utility
6. A Allocation handler which distibutes uploaded trajectories to detected vehicles

> **Note:** this might take a while (can be up to 30 min or an hour depending on internet) on first run as downloads are required.

> **Note:** Use ctrl+c to stop the process when you have finished.

The User Interfaces are available in the following locations:

- Go to [`http://localhost:8080`](http://localhost:8080) in a browser to (hopefully) see the gazebo simulator.
- Go to [`http://localhost:3000`](http://localhost:3000) in a browser to see the starling user interface containing go/stop buttons.

> **Note:** All specified sites can be accessed from other machines by replacing `localhost` with your computer's IP address.

> **Note:** Sometimes it might take a bit of time for the UIs to become available, give it a minute and refresh the page. With Gazebo you may accidentally be too zoomed in, or the grid may not show up. Use the mouse wheel to zoom in and out. The grid can be toggled on the left hand pane.

## Offboard Control
There are four supported methods for offboard control of either the SITL or real drones.

1. Control drone directly via Mavlink, by Ground Control Station (GCS) or other Mavlink compatible method (e.g. Dronekit).
2. Control drone via Custom ROS2 node
3. Control drone via Simple Offboard ROS2 node
4. Uploading a trajectory via the GUI on [`http://localhost:3000`](http://localhost:3000)

### 1. Connecting a Ground Control Station via Mavlink

If a mavros or sitl instance is running, there will be a GCS link on `udp://localhost:14553` (hopefully). This means that you can run a GCS such as QGroundControl or Mission Planner:

- Create a comms link to `localhost:14553` on UDP
- The GCS should auto detect the drone(s)
- You should be able to control and monitor any sitl drone through the standard mavlink interface.

This is a quick an easy way to control the SITL instance via Mavlink.

### 2. Running Example ROS2 Offboard Controller node

An example offboard ROS2 controller can then be conncted to SITL by running the following in a separate terminal:

```bash
docker pull uobflightlabstarling/example_controller_python # Download the latest container
docker run -it --rm --network projectstarling_default uobflightlabstarling/example_controller_python
```
This will download and run the `example_controller_python` image from `uobflightlabstarling` on docker hub. `-it` opens an interactive terminal. `--rm` removes the container when completed. `--network` attaches the container to the default network created by `docker-compose`. The default network name is `<foldername>_default`.

> **Note:** The controller may complain that it cannot find the drone. Double check that the name of the root folder matches the one passed to `--network`.

When run, the example will confirm in the terminal that it has connected and that it is waiting for mission start. To start the mission, press the green go button in the [starling user interface](http://localhost:3000) which will send a message over `/mission_start` topic. A confirmation message should appear in the terminal, and the drone will arm (drone propellors spinning up) and takeoff. It will fly in circles for 10 seconds before landing and disarming.

### 3. Using ROS2 Simple Offboard Controller Node

If running the 'Full' version instructions above, this node will already be running. If not runing, the container can be run by running the following in a separate terminal:

```bash
docker pull uobflightlabstarling/starling_simple_offboard # Download the latest container
docker run -it --rm --network projectstarling_default -e VEHICLE_MAVLINK_SYSID=1 uobflightlabstarling/starling_simple_offboard
```

This will download and run the simple offboard controller node. This node provides a number of [ROS2 Services](https://docs.ros.org/en/foxy/Tutorials/Services/Understanding-ROS2-Services.html) which can be called to interact with the connected drone. The easiest way to interact with this controller is by uploading a trajectory for the integrated trajectory follower to handle (see next point). See the [repository](https://github.com/mhl787156/starling_simple_offboard) for further details.

### 4. Uploading a Trajectory

If running the 'Full' version instructions above, the GUI should be available on [`http://localhost:3000`](http://localhost:3000). Navigating to  [`http://localhost:3000/load_trajectories`](http://localhost:3000/load_trajectories) will give you a page where you can upload trajectories in csv or excel format. The trajectories can be position, velocity, attitude or attitude rates. Click on the 'Help' button for more information on trajectory format.

## Onboard Control
In this instance there is only an abstract difference between onboard and offboard control since all controllers are running on the same machine as the simulated vehicle.

## Implementing a Controller
### Modifying the example controller
In the [controllers](https://github.com/UoBFlightLab/ProjectStarling/tree/master/controllers) folder there is an example_controller_python which you should have seen in action in the example above. The ROS2 package is in [example_controller_python](https://github.com/UoBFlightLab/ProjectStarling/tree/master/controllers/example_controller_python/example_controller_python). Any edits made to the ROS2 package must first be built:
```bash
cd controllers
make example_controller_python
```

>**Note** Alternatively you can run the build manually through `docker build -t example_controller_python example_controller_python/` (i.e. point to the folder containing the example_controller_python dockerfile)

This will use colcon build to build the node and output a local image named `example_controller_python`. This local image can be run as follows:

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
2. A Dockerfile (named `Dockerfile`) which is dervied `FROM uobflightlabstarling/starling-controller-base`, use the [example Dockerfile](https://github.com/UoBFlightLab/ProjectStarling/blob/master/controllers/example_controller_python/Dockerfile) as a template.


Your Dockerfile can be built by running the following in the directory with the Dockerfile.
```bash
docker build -t <name of your controller> <folder containing the dockerfile>
# e.g. docker build -t my_new_controller .
```

Your container can then be run as above using `docker run.

## Troubleshooting/ FAQs