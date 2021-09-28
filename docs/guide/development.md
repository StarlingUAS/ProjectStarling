# Developing your own containers

This guide shows recommended methods for developing your own controllers and using Starling to develop your exeperiments.

## Contents

[TOC]

## Development Environment

We recommend that the any user looking to do development uses [Visual Studio Code](https://code.visualstudio.com/).
VScode is a free cross platform, general use text editor and development environment with a vast array of built in tools.
In particular we recommend VScode for use with Starling as it has a number of easy to use extensions suited for [remote development](https://code.visualstudio.com/docs/remote/remote-overview) which includes

* Development over SSH (i.e. developing onboard a drone)
* Development inside a local docker container
* Development inside a remote docker container (e.g. a container running on a Raspberry Pi connected over SSH)!

## Developing a container

There are actually a number of methods of developing an application within a containerised environment.

1. Developing fully outside of the container and then building only using docker build.
    * This is the simplest and most straight forward method, recommended for most applications.
    * Iteration can prove to be a little slow as docker build attempts to rebuild the entire container.
2. Creating/pulling the project outside of the container and bind mounting the project into the container and building within.
    * This can be achieved by using the bind syntax: `docker run -itd --name <container reference name> -v $(pwd)/<my local files>:/ros_ws/src/ <containername>`. Ommitting the `--rm` will stop the container from being automatically removed, and therefore the container will always be available/save your working environment.
    * This essentially places your local files into the docker container for the container to interact with (read/write/execute).
    * The local files can then be viewed and edited by using the docker remote development tool in vscode, then run via the integrated terminal (remember to source ros using `source /opt/ros/foxy/setup.bash`).
    * Any local changes can be built using standard ROS2 procedures (i.e. `cd /ros_ws && colon build packages-select <my package name>`)
    * Iteration is very quick as changes can be made and built/tested from within the container.
    * Any local changes can then be pushed to github/repositories in the usual manner. Note that files created inside the container may have permissions issues, these can be solved by running `sudo chown -R <user>:<user> <my project>`.
    * Container can be restarted if the container enviornment is not quite right (e.g. need more ports or different volume mounted or need different network configuration)
    * Recommended method for more complex or detailed applications.
3. Running a Docker container (such as `uobflightlabstarling/starling-controller-base`) and creating/pulling the project fully inside the container.
    * Start the container *without the --rm*, and giving it a permanent name using `--name <container name>`. This will ensure the container is persistent.
    * Use VScode to remote inside the container
    * Using the integrated terminal (or by using `docker exec ...`) install `git` and any other useful libraries or tools
    * Pull or start your project within the ros workspace (`/ros_ws`)
    * Any local changes can be built using standard ROS2 procedures (i.e. `cd /ros_ws && colon build packages-select <my package name>`)
    * Iteration is very quick and your local environment need not be modified in any way.
    * Be aware that you may need to repeat some of these steps after restarting the container if the container environment (e.g. ports/networking) needs to be changed.
    * Recommended method if quick changes need to be tested, but not for full development in case the environment needs to be changed.
