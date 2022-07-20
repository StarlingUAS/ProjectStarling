# zenoh_bridge

Zenoh DDS Bridge

This is a UAV project involving PX4 and ROS2. 

## Installation
### Pre-requisits
This project uses Starling as its base. Therefore you will need thos prerequisites, please see [Starling documentation](https://docs.starlinguas.dev). This is primarily docker. You do not need to clone this project.

### UAV simulator

To test this locally, you can run the simulation stack using docker-compose specified in the dokcer-compose file in the deployment directory

```
docker-compose -f deployment/docker-compose.yml up
```

## Basic Usage

### Run the docker container
If the container has been pushed to the cloud, the most straightforward method is using docker in conjunction with the docker-compose scripts in ProjectStarling or Murmuration

```bash 
docker run -it --rm --net=bridge mickeyli789/zenoh-bridge:latest
```

### Building the docker container locally
First you will need to recursively clone the repo into your workspace, cd into the directory and then run `make`:
```
git clone --recursive https://github.com/mhl787156/zenoh_bridge.git
cd zenoh_bridge
make build
```

## Running in Simulation

### Starling CLI
his project then uses the [Murmuration project](https://github.com/StarlingUAS/Murmuration) in order to simplify the multi-vehicle control and simulation. Murmuration includes a Command Line interface and example scripts to help run multi-vehicle simulations. In particular the structure of these simulations directly matches the real life deployment within the flight arena.

You will need to clone the Murmuration project somewhere into your workspace, recommendation is in the parent directory to this one.
```
git clone https://github.com/StarlingUAS/Murmuration.git
```

Following the instructions from Murmuration, it is recommended you add the Murmuration file path to your PATH. Either run in each terminal:
```
export PATH="$(pwd)/bin":$PATH
```

or add the following to the bottom of your bashrc/zshrc file:
```
export PATH="<path to Marsupial root folder>/bin":$PATH
```

then with the Starling CLI on the path, you should be able to run
```
starling install
```

### Running the simulation

First start the simulated cluster (here we start an example with 2 vehicles)
```
starling start kind -n 2
starling simulator start --load
```

After locally building the container (running `make`) you will need to upload it to the simulator
```
make
starling utils kind-load uobflightlabstarling/position-trajectory-controller:latest
```

Optionally, you can also load in the allocator and the dashboard
```
starling utils kind-load uobflightlabstarling/starling-allocator:latest
starling utils kind-load mickeyli789/starling-ui-dashly:latest
```

Finally, you can start this simulation by running
```
starling deploy -f kubernetes
```

To stop or restart the simulation, you can run:
```
starling deploy -f kubernetes stop
starling deploy -f kubernetes restart
```

Then you can open a browser page at https://localhost:8080 for gazebo and https://localhost:3000 for the user interface page.

You may also want to start the dashboard using the following command which opens on https://localhost:31771
```
starling start dashboard
```

## Project Details

Fill in your project detals here

## License

This project is covered under the MIT License.