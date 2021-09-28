# Getting Started with Starling

## Contents
[TOC]

## Installation

The only dependency of running and using Starling locally is the [Docker] containerisation software. Please visit [docker's installation page](https://docs.docker.com/get-docker/) for further details. We recommend using Starling on linux based systems, but it should work on Windows and possible Mac depending on the state of the Docker.

### Linux/ Ubuntu

For Linux systems, see the following [install page](https://docs.docker.com/engine/install/ubuntu/). There are multiple ways of installation docker, but we recommend installing using the repository method:

1. Update the `apt` repository and install requirements

        sudo apt-get update

        sudo apt-get install \
            apt-transport-https \
            ca-certificates \
            curl \
            gnupg \
            lsb-release

2. Add Docker's official GPG key:

        curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /usr/share/keyrings/docker-archive-keyring.gpg

3. Install Docker (and docker-compose!):

        sudo apt-get update

        sudo apt-get install docker-ce docker-ce-cli docker-compose containerd.io

4. Test Docker installation:

        sudo docker run hello-world

This will install Docker, accessible using `sudo` root privileges only. To use docker without sudo, run the following (be aware of [issues with this](https://docs.docker.com/engine/install/linux-postinstall/#manage-docker-as-a-non-root-user))

1. Run the following

        sudo groupadd docker
        sudo usermod -aG docker $USER

2. Log out and log in again to enforce changes
3. Verify that it was successful:

        docker run hello-world

That is Docker on Linux installed. See [the original page](https://docs.docker.com/engine/install/ubuntu/) for any further details.

### Windows

For Windows systems, please see [the windows installation instructions for full details](https://docs.docker.com/docker-for-windows/install/). You will need to install Docker Desktop for Windows.

Starling has not been fully tested on Windows, but single-agent non-cluster (docker-only) applications should be compatible.

### Mac OS

For Mav systems, please see [the Mac Os installation instructions](https://docs.docker.com/docker-for-mac/install/) for full details. You will need to install Docker Desktop for MAC OS.

Starling has not be tested on MAC OS, so there is no guarantee that it is functional. However in theory the single-agent non-cluster (docker-only) application should be compatible.

## Using Starling

Starling provides a framework for users to implement almost any drone related experiment or application. However, we have identified that most applications fall into the following categories. Therefore we have provided detailed user guides for these use cases. It is recommended that the user follow these guides to run starling before starting your own implementation as these guides will also slowly introduce you to the concepts and details Starling uses.

These user guides are all located on the side navigation bar for easy access.

A key idea is the fact that the controller you develop, whether onboard or offboard, will be identical regardless of which step you are in. This therefore designs a workflow for the development of controllers.

### Single Drone Applications

For *single drone* applications, there are 3 steps to the development of a controller running onboard (on the 'drone' itself) or offboard (on a separate machine):

1. **Step 1:** *Single Drone testing on local machine using Docker only*

    - Follow the following [user guide](../single-drone-local-machine)
    - Simplest deployment method and the quickest way to get a simulation (gazebo) and drone (px4, mavros) instance running.
    - Quick and easy way to test the basic functionality of your controller without worrying about too may other factors.
2. **Step 2:** *Single Drone testing on local cluster*
    - Follow the following [user guide](../kube-single-drone-local-machine)
    - Once you have your controller mostly working and debugged, the next step is to test it within the network and system architecture used when flying real drones.
    - This will require learning a little bit about the cluster architecture and how Starling works under the hood.
3. **Step 3:** *Single Drone flying at the Robotics Laboratory*
    - Follow the following [user guide](../single-drone-drones)
    - Detailing how to run your controller on real drones at the Bristol Robotics Laboratory

### Multiple Drone Applications

For *multiple drone* applications, there are also 3 steps to the development of a centralised or decentralised, onboard or offboard controller.

1. **Step 1:** *Single Drone testing on local machine using Docker only*
    - Follow the following [user guide](../single-drone-local-machine)
    - Simplest deployment method and the quickest way to get a single simulation (gazebo) and drone (px4, mavros) instance running.

    - Quick and easy way to test the basic functionality of your controller without worrying about too may other factors.
    - Can be used to quickly protype controllers before testing with multiple vehicles.
2. **Step 2:** *Multiple Drone testing on local cluster*
    - Follow the following [user guide](../multiple-drone-local-machine.md)
    - The cluster facilitates the multi-agent capability of starling.
    - The guide takes you through the basic concepts required for running multi-agent applications
3. **Step 3:** *Multiple Drones flying at the Robotics Laboratory*
    - Follow the following [user guide](../controllers/example_controller_python/Dockerfileguide/multiple-drone-drones)
    - Detailing how to run your controller on real drones at the Bristol Robotics Laboratory


## FAQs
