# Simulator
## Running the simulator image

At the moment, usinga graphical simulator and passing through Xserver from host.
For OpenGL stuff need to have nvidia-container-toolkit installed on the host:
https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html

In summary add nvidia docker repos:
```sh
distribution=$(. /etc/os-release;echo $ID$VERSION_ID) \
   && curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add - \
   && curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list
```

Install
```sh
sudo apt update
sudo apt install nvidia-docker2
sudo systemctl daemon-reload
sudo systemctl restart docker
```

## Build & Run

Use `./build_images.sh` to build the docker images
Use `./run_docker.sh` to start the clover2 simulator

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

See [this README.md for more details](controller/README.md)