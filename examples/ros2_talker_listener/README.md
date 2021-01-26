# Controller

This controller is designed to run in a single container with the aim that it will run aboard the drone. It will contain all the necessary dependencies to run custom code.

All is within the [controller](controller/) directory. Within, there is a Makefile which automates the task of building the containers.

To run:
```
cd controller
make 
make run_controller
```

There are two docker files:

- `Dockerfile.mavros` is the base container which is based on Ubuntu 20.04. It runs ROS2 Foxy, ROS1 Noetic and MAVROS via the ROS1_bridge.
- `Dockerfile.controller` is an example controller file which is based on `Dockerfile.mavros`. It is currently set up to
   1. Create a new user `ctrl_user`
   2. Copy the contents of the `workspace` directory
   3. Run `workspace/setup_script.sh` inside the container (currently this builds any ROS1/2 packages within `workspace`)
   4. Run `workspace/launch.sh` 

## Base Container (Mavros)

We have identified that mavros is a base requirement for any drone running onboard compute in order to communicate with both the autopilot and GCS. 