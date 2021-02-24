# Deployment with kubernetes (k8)

This system is intended to run as a cloud platform. We utilse [k3s](https://rancher.com/docs/k3s/latest/en/quick-start/) as our kubernetes manager. 

Key concepts are as follows:
- **pods** are a k8 concept. A pod contains one or more containers and has its own ip address. Containers within a pod communicate over localhost
- **node** is the machine (physical e.g. pi or virtual machine) upon which pods are run. (Separate from 'ros2 nodes' or 'ros nodes')
- **cni** container networking interface (default is flannel for k3s) is the underlying networking for all containers
- **dds/ fast-rtps** Is the default communications middleware for ros2 comms.

Refer to the kubernetes notes with the onenote notebook for more usage information.

## Quick Reference to files:
The .yaml files in this directory are all kubernetes configurations for various combinations of systems. The cpu architecture refers to where the containers have been specified to run - amd64 specifies for running on master machine, and arm64 specifies for running on the raspberry pi node over the network (see below for setup). All of these config files pull their images from the [uobflightlabstarling docker hub](https://hub.docker.com/orgs/uobflightlabstarling/repositories).

### Files

- **k8.starling-gazebo.amd64.yaml** :- Currently runs the `starling-sim-iris` image and a Service which exposes the gzweb statically on `localhost:8080`. This service has a cluster internal hostname of `sim-gazebo.gazebo-srv`
- **k8.px4-sitl.amd64.yaml** :- Currently runs a pod containing two containers
    - `starling-sim-px4-sitl` - emulating px4-sitl. Talks to GCS software on port 14550 with replies on 18570.
    - `starling-mavros` - contains a ROS2 mavros node connected via udp://localhost:14540 to sitl. Talks to GCS on udp broadcast port 14553.
- **k8.ap-sitl.amd64.yaml** :-  [Needs updating]Currently runs a pod containing two containers
    - `starling-ardupilot-sitl` - emulating ardupilot-sitl. Talks to GCS over 14553 as well. 
    - `starling-mavros` - contains a ROS2 mavros node connected via tcp://localhost:5762 to sitl. Talks to GCS on udp broadcast port 14553.
- **k8.mavros.arm64.yaml** :- A mavros node designed to run on the raspberry pi/ drone control computer. This pod contains a single `starling-mavros` container. It reads of a px4 pixhawk assumed to be talking over usb serial connection `/dev/px4fmu` (set up via udev symlinks). Currently assumes mavlink sysid is 1.
- **k8.ros_monitor.amd64.yaml** :- runs `starling-mavros` and a network-tools container. Can be used for debugging ROS2 and networking issues

### Using files
Once k3s has been installed (see below, or run `./run_k3s.sh` in the home directory), these configurations can be used in the cluster as follows:
```bash
# Applying/ Creating them
sudo k3s kubectl apply -f <filename.yaml> 
# Deleting the deployment
sudo k3s kubectl delete -f <filename.yaml> -f <filename.yaml>
```
This can also be done in the gui dashboard application.

> **Note:**
> Local images can be used if `imagePullPolicy` is set to `ifNotPresent` instead of `Always`. In that case it will attempt to find a local image with the given image name.
> arm64 images must be cross compiled using docker buildx (make multi-arch or make cross-compile or similar in the relevant docker files) and always pulled from docker hub.

## Installation instructions

Install k3s using the install script, this will fetch k3s and run the kubernetes master node immediately:
```
curl -sfL https://get.k3s.io | INSTALL_K3S_EXEC="--docker" sh -
```

> Once installed, the equivalent instruction is `sudo k3s server --docker`

For the raspberry pi, ensure docker is installed, and then these instructions are the same on the raspberry pi (64 bit os).

For testing purposes (inside testing dir), the containers have already been built for both amd64 and arm64 and uploaded onto hub.docker: [mickeyli789/ros_demo](https://hub.docker.com/r/mickeyli789/ros_demo).

Also recommended you alias kubectl (kubernetes cli) in your bashrc
```
alias kubectl='sudo k3s kubectl
```

## Running instructions

### Laptop

To run the master kubernetes server using docker (instead of containerd if you need access to local images). In one terminal run either:
1. '```sudo k3s server --docker```' (will start in local terminal)
2. '```curl -sfL https://get.k3s.io | INSTALL_K3S_EXEC="--docker" sh -``` ' (will run in background as systemd - check `systemctl status k3s`)

This will open up a server with entrypoint on `0.0.0.0:6443` which corresponds to `<host local ip address>:6443` 

### Pi / Drone / Agent
First ensure that the pi has been correctly set up with an airgapped installation of k3s, [see here for installation instructions](https://rancher.com/docs/k3s/latest/en/installation/airgap/). Follow the Manually Deploy Images Method.

#### Setup script via ssh

Identify the ip address of the pi, the root enabled (possibly password disabled) username. Then from this directory run
```bash
./start_k3s_agent.sh <remote username> <remote ip address> <node name>
```
e.g.
```bash
./start_k8_agent.sh ubuntu 192.168.0.110 clover1
```
You can specify the k3s server address by setting the environment variable before calling:
```bash
K3S_SERVER=https://192.168.0.63:6443 k3s_agent ubuntu 192.168.0.96 raspi1
```

#### Manual, old setup method.

First SSH onto the pi

First ensure you run `k3s-killall.sh` to make sure there is no master server running as you only want `k3s agent` to run.

The K3S_TOKEN is the contents of the file `/var/lib/rancher/k3s/server/node-token`

```bash
K3S_TOKEN=<contents of the file /var/lib/rancher/k3s/server/node-token>
#e.g. K3S_TOKEN=K103b62838822f40f3e41j51f10cb127236f2c3014c120ede19263da9f33fbfc859::server:2dcbb32a4cad16e20d714d88dbce4af8
K3S_SERVER=https://<Your main machine ip address>:6443
K3S_NODE_NAME=clover1

echo "Killing all k3s services and instances first"
k3s-killall.sh

echo "Starting k3s agent only"
sudo k3s agent -t ${K3S_TOKEN} -s ${K3S_SERVER} --node-name ${K3S_NODE_NAME}
```
The Pi should now be setup

Consider running the above using screen or somehow in the background just in case your ssh connection is unstable or you want to close it. 

### Post actions

If you want to stop kubernetes completely, the internet install script comes with two options which are on the PATH and can be run in the terminal.
1. `k3s-killall.sh` will stop all k3s nodes and the systemd
2. `k3s-uninstall.sh` will delete everything k3s and remove the systemd

#### Dashboard
[See the k3s docs for info on how to run](https://rancher.com/docs/k3s/latest/en/installation/kube-dashboard/)

Are started automatically in the `./run_k3s.sh` script.
## Running the test cases

Go to [testing directory for more info](testing/README.md)



