# Single Drone Local Machine

Follow these instructions for quick and easy testing of controllers on a single drone on a single local machine. **Use in the following scenarios**:

1. Local development of single drone applications in simulation (gazebo)
2. Local development of offboard (drone control software on pc not on drone)
3. Local development of onboard (drone control software running on drone itself)

## Contents
[TOC]

## Starting the Drone and Simulator 

In the root directory, execute `make run` (or `docker-compose up`) in a terminal. This will start the following:

1. Gazebo simulation enviornment running 1 Iris quadcopter model 
2. A SITL (Software In The Loop) instance running the PX4 autopilot.
3. A Mavros node connected to the SITL instance 
4. A simple UI with a go and estop button. 

> **Note:** this might take a while on first run as downloads are required.

The User Interfaces are available in the following locations:

- Go to [`http://localhost:8080`](http://localhost:8080) in a browser to (hopefully) see the gazebo simulator.
- Go to [`http://localhost:3000/html/main.html`](http://localhost:3000/html/main.html) in a browser to see the starling user interface containing go/stop buttons.

> **Note:** All specified sites can be accessed from other machines by replacing `localhost` with your computer's IP address. (TODO: starling ui not yet functioning in this use case.)

> **Note:** Sometimes it might take a bit of time for the UIs to become available, give it a minute and refresh the page. With Gazebo you may accidentally be too zoomed in, or the grid may not show up. Use the mouse wheel to zoom in and out. The grid can be toggled on the right hand pane.  

## Offboard Control
{% include 'guide/offboard-control.md' %}

## Onboard Control
{% include 'guide/onboard-control.md' %}

## Troubleshooting/ FAQs