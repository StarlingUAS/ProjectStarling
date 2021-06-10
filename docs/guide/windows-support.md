# Starling on Windows

Running Starling on Windows needs a few things to be installed:

## Installing Docker

Starling relies on a technology called Docker. This has a Windows installer on the docker site:
[docs.docker.com/docker-for-windows/install/](https://docs.docker.com/docker-for-windows/install/)

Download and run the installer, to install Docker on Windows. Note that if you have Windows 10 Home
(rather than Pro), the instructions differ slightly. They can be found here:
[docs.docker.com/docker-for-windows/install-windows-home/](https://docs.docker.com/docker-for-windows/install-windows-home/).

## Installing Git

If you don't already have Git installed, it is *highly* recommended. Git manages source code
versioning and is used for distributing the core Starling files. It can be downloaded from here:
[git-scm.com/download/win](https://git-scm.com/download/win). You could also install GitHub Desktop
(other vendors are available).

## Getting Starling

Clone the `ProjectStarling` repo to a working folder on your computer. You probably want to avoid
any cloud-backed folders as these tend to interfere with Git.

With Git installed as above, you can open PowerShell, navigate to the folder where you want to
clone the project (using `cd`) and run:

```
git clone https://github.com/UoBFlightLab/ProjectStarling
```

## Running

With Docker installed and the repo downloaded, navigate into the project folder in a terminal.
Once there you should see a file called `docker-compose.tcp.yml`. This contains instructions for a
tool called Docker Compose to setup a set of containers for you. Note that this version has been
modified slightly to use TCP ports which makes things easier on Windows/WSL.

To launch the containers, open PowerShell, make sure you're in the root of the ProjectStarling repo
and run:
```ps
docker compose -f docker-compose.tcp.yml up
```

You should see the tool begin to "pull" (download) the files needed to run the project. These have
been built from files in the Starling repo and uploaded to DockerHub to save you time. Some of these
files are quite big, so be prepared to wait a while for downloading to finish. After the first run,
the system will use the already-downloaded files which will speed up the process.

One the downloading process is complete, the tool will begin to setup the containers. At this point,
Windows may prompt you to allow "Docker Container Backend" to connect to networks

With everything running, and docker allowed to access the network, open a web browser and go to
[localhost:8080](http://localhost:8080) where you should see the web-based Gazebo interface.

You can also launch Mission Planner (other GCS software is available) and connect to TCP port 5760.
Using the IP address of `127.0.0.1` should work as docker will ensure that the connection gets
routed to the right place.