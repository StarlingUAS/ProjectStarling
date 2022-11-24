# Docker

## Docker Documentation
For an introduction to Docker & containers, see the [official docs](https://www.docker.com/resources/what-container).

## Containerisation

With traditional methods, code is developed in a specific computing environment which, when transferred to a new location, often results in bugs and errors. For example, when a developer transfers code from a desktop computer to a virtual machine (VM) or from a Linux to a Windows operating system. This is especially true when developing systems for hardware. For drones in particular,  existing UAV software development often involved a lot of system specific solutions tied to the hardware. For example, most drones and controlling computers are configured only for their designated applications, tying both computer and drone to that application. It also meant that offline testing, and testing away from the physical drone was impossible, hampering the development process as well.

Containerisation eliminates this problem by bundling the application code together with the related configuration files, libraries, and dependencies required for it to run. This single package of software or “container” is abstracted away from the host operating system, and hence, it stands alone and becomes portable—able to run across any platform or cloud (UAV included), free of issues.

Under the hood, containerisation is a form of system virtualisation where applications are run in isolated user spaces. This is also why they are often referred to as lightweight as containerisation does not virtualise the entirety of an operating system for each application, instead allowing for containers to share use of the host machine's operating system kernel. Because of this, containers are inherently smaller in capacity than a full VM and require less start-up time and system resources. In essence each container can be thought of simply as an easy to use 'executable' which is run by a runtime engine, in our case Docker. In addition the abstraction from the host operating system makes containerised applications portable and able to run uniformly and consistently across any platform or cloud. Containers can be easily transported from a desktop computer to a virtual machine (VM) or from a Linux to a Windows operating system, making it perfect for software development on any machine.

## DockerFiles

A Docker container is specified by the user using the definition of a Dockerfile. An example Dockerfile is given here in this code snippet.

```dockerfile
FROM ${REGISTRY}uobflightlabstarling/starling-controller-base:${VERSION}
COPY example_controller_python /ros_ws/src/example_controller_python
RUN . /ros_ws/install/setup.sh && colcon build
CMD [ "/bin/bash" ]
```

 A Dockerfile is then **built** to create an executable which can then be run.

 Note the difference between build time commands and runtime commands.

 A built Dockerfile comprises of layers, where, in general, each command specified in the Dockerfile describes a layer. Our example describes a container image with 4 layers, one for each command. This is important because layers can be downloaded once and shared between among multiple containers, again reducing runtime overhead - especially on resource constrained platforms such as the companion computers on UAVs.