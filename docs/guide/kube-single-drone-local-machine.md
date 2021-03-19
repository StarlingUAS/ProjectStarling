# Single Drone Local Machine Emulating the Actual Drone Software/Communications Architecture

Follow these instructions for quick and easy testing of controllers on a single drone on a single local machine. **Use in the following scenarios**:

1. Local development of single (or multiple) drone applications in simulation (gazebo)
2. Local development of offboard (drone control software on pc not on drone)
3. Local development of distributed onboard (drone control software running on drone itself)
4. Testing controllers/software on the real drone software/ communications architecture that would be used in the BRL.

**This is considered to be step 2 for the Starling development process.**

> **NOTE:** Reading the [background](../details/background.md) may be useful but not necessary.

## Contents
[TOC]

## Drone and Simulator on a local cluster

### Starting the cluster

In the root directory run `./run_k3s.sh` in a terminal. This will start the following:

1. The cluster root node which governs the running of all the other parts.
2. Gazebo simulation enviornment running 1 Iris quadcopter model 
3. A SITL (Software In The Loop) instance running the PX4 autopilot.
4. A Mavros node connected to the SITL instance 
5. A simple UI with a go and estop button. 
6. The cluster control dashboard (printing out the access key string)

> **Note:** this might take a while on first run as downloads are required.

The User Interfaces are available in the following locations:

- Go to [`http://localhost:8080`](http://localhost:8080) in a browser to (hopefully) see the gazebo simulator.
- Go to [`http://localhost:3000/html/main.html`](http://localhost:3000/html/main.html) in a browser to see the starling user interface containing go/stop buttons.
- Go to [`http://localhost:31771`](http://localhost:31771) in a browser to see the cluster dashboard. There is a lot here, and this guide will point you to the key functions. Please see [this page](../details/kubernetes-dashboard.md) for further details.

> **Note:** All specified sites can be accessed from other machines by replacing `localhost` with your computer's IP address. (TODO: starling ui not yet functioning in this use case.)

> **Note:** Sometimes it might take a bit of time for the UIs to become available, give it a minute and refresh the page. With Gazebo you may accidentally be too zoomed in, or the grid may not show up. Use the mouse wheel to zoom in and out. The grid can be toggled on the right hand pane.  

### Offboard Control
There are two supported methods for offboard control of either the SITL or real drones.

1. Control drone directly via Mavlink, by Ground Control Station (GCS) or other Mavlink compatible method (e.g. Dronekit).
2. Control drone via ROS2 node

### 1. Connecting a Ground Control Station via Mavlink

If a mavros or sitl instance is running, there will be a GCS link on `udp://localhost:14553` (hopefully). This means that you can run a GCS such as QGroundControl or Mission Planner:

- Create a comms link to `localhost:14553` on UDP 
- The GCS should auto detect the drone(s) 
- You should be able to control and monitor any sitl drone through the standard mavlink interface. 

This is a quick an easy way to control the SITL instance via Mavlink.

### 2. Running Example ROS2 Offboard Controller node


### Onboard Control
{% include 'guide/onboard-control.md' %}



## Development on the Drone and Simulator 

There are a number of useful scripts in the `/scripts` directory of this repository. Scripts can be run from any location, but for this tutorial we assume the user is in the root directory.





### Starting the cluster root node


## Troubleshooting/ FAQs