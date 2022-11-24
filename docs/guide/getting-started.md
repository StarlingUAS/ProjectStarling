# Getting Started with Starling

## Contents
[TOC]

## Install Docker

The only dependency of running and using Starling locally is the [Docker] containerisation software. Please visit [docker's installation page](https://docs.docker.com/get-docker/) for further details. We recommend using Starling on linux based systems, but it should work on Windows and possible Mac depending on the state of the Docker.

### Linux/ Ubuntu

#### Install Script
For linux, we have provided a helpful install script. First you will need to clone the Murmuration github repository:
```
git clone https://github.com/StarlingUAS/Murmuration.git
```

Then run:
```
sudo ./starling install
```

Once that completes you should be all good to go for the next step.

#### Manual Installation
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

2. Log out and log in again to enforce changes (may need to restart)
3. Verify that it was successful:

        docker run hello-world

That is Docker on Linux installed. See [the original page](https://docs.docker.com/engine/install/ubuntu/) for any further details.

### Windows

For Windows systems, please see [the windows installation instructions for full details](https://docs.docker.com/docker-for-windows/install/). You will need to install Docker Desktop for Windows.

Follow the instructions and tutorial below, but refer to our [Windows Support page](windows-support.md) if you have issues.

Starling has been fully tested on Windows, and both single and multi-agent cluster testing is available through the use of the Starling CLI. However it is recommended that you use WSL rather than powershell.

### Mac OS

For Mav systems, please see [the Mac Os installation instructions](https://docs.docker.com/docker-for-mac/install/) for full details. You will need to install Docker Desktop for MAC OS.

Starling has not be tested on MAC OS, so there is no guarantee that it is functional. However in theory the single-agent non-cluster (docker-only) application should be compatible.

## Get Starling

For simple usage, or for usage as a simulator, you can use the Murmuration repository which gives a number of starling examples and a command line interface.

```
git clone https://github.com/StarlingUAS/Murmuration.git
```

For more advanced usage, or to see examples of the core elements, clone the Starling repository:
```
git clone https://github.com/StarlingUAS/ProjectStarling.git
```

## Using Starling

Starling provides a framework for users to implement almost any drone related experiment or application. However, we have identified that most applications fall into the following categories. Therefore we have provided detailed user guides for these use cases. It is recommended that the user follow these guides to run starling before starting your own implementation as these guides will also slowly introduce you to the concepts and details Starling uses.

These user guides are all located on the side navigation bar for easy access.

A key idea is the fact that the controller you develop, whether onboard or offboard, will be identical regardless of which step you are in. This therefore designs a workflow for the development of controllers.

For those with no prior programming experience and/or wish to flyg a single drone outdoors (using the ardupilot flight stack), start with the [first tutorial](../../tutorials/introduction). 

For those who have done the first tutorial or wish to fly indoors with single or multiple vehicles (using the PX4 flight stack), follow on with the [second tutorial](../../tutorials/introduction_px4)

## FAQs
