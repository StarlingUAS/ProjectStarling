# Useful and Example Controllers developed for general starling usage

This folder contains a number of general controllers for general usage of the starling system.

## Building and Installation

This folder can be built locally by running `make all` from this folder. Each element can also be built separately by invoking `make <controller-name>`.

## Example Python Controller

This is an example of a controller for the system written in Python with ROS2. It has been packaged up with a Dockerfile and an example kubernetes deployment file. The controller itself directly talks to mavros and tells the drone to lift off, trace a semi circle and land.

## Starling Simple Offboard

This is an example of an onboard or offboard controller which can be used to control a single drone. It is based on the original Clover simple offboard, but also includes a trajectory follower module. This controller can be run to provide a higher level control of a real or simulated drone.

> **Note** If running in the flight arena, this controller is already running on the Coex Clover drones

## Starling Allocator

This is an example of a trajectory allocator node which attempt to smartly allocate trajectories to drones which can run them.