# Multiple Drones Local Machine


Follow these instructions for quick and easy testing of controllers on multiple simulated drones on a single local machine. **Use in the following scenarios**:

1. Local development of multiple drone applications in simulation (gazebo)
2. Local development of offboard control (drone control software on pc not on drone)
3. Local development of distributed onboard control (drone control software running on drone itself)
4. Testing controllers/software on the real drone software/ communications architecture that would be used in the BRL.

**This is considered to be step 2b for the Starling development process.**

> **Note:** Reading the [background](../details/background.md) may be useful but not necessary.

## Contents
[TOC]

## Drone and Simulator on a local cluster
### Starting the cluster

In the root directory run one of the following in a terminal
```bash
./run_k3s.sh
./run_k3s.sh -ow # Will automatically open the UI webpages for you. 
``` 
This will start the following:

1. Install the cluster root node which governs the running of all the other parts.
2. A node running the Gazebo simulation environment 
3. A node running the following:
    1. An initialisation routine spawning 1 Iris quadcopter model 
    2. A SITL (Software In The Loop) instance running the PX4 autopilot.
    3. A MAVROS node connected to the SITL instance 
4. A simple UI with a go and estop button. 
5. The cluster control dashboard (printing out the access key string)

> **Note:** this might take a while on first run as downloads are required.

The User Interfaces are available in the following locations:

- Go to [`http://localhost:8080`](http://localhost:8080) in a browser to (hopefully) see the gazebo simulator.
- Go to [`http://localhost:3000/html/main.html`](http://localhost:3000/html/main.html) in a browser to see the starling user interface containing go/stop buttons.
- Go to [`http://localhost:31771`](http://localhost:31771) in a browser to see the cluster dashboard. There is a lot here, and this guide will point you to the key functions. Please see [this page](../details/kubernetes-dashboard.md) for further details.
    - Your browser of choice may not like the web-page and complain of certificate errors. Please ignore this and continue onwards. You may have to click 'advanced' or a similar option for the browser to let you in.
    - To log in to the site, you will need the long login Token which is hopefully displayed by `run_k3s.sh`. This token should also be automatically placed onto your clipboard for pasting.

> **Note:** All specified sites can be accessed from other machines by replacing `localhost` with your computer's IP address. (TODO: starling ui not yet functioning in this use case.)

> **Note:** Sometimes it might take a bit of time for the UIs to become available, give it a minute and refresh the page. With Gazebo you may accidentally be too zoomed in, or the grid may not show up. Use the mouse wheel to zoom in and out. The grid can be toggled on the left hand pane.  

### Restarting or deleting the drone or simulator in the cluster

There may be cases where you wish to restart or refresh either the software running on the drones or the simulator itself (e.g. removing old models):
```bash
./run_k3s.sh -d # or --delete will remove the gazebo and drone instances
./run_k3s.sh -r # or --restart will restart the gazebo and drone instances
```

If you wish to remove the cluster and all associated software from the machine, you will need to uninstall:
```bash
./run_k3s.sh --uninstall 
```
> **Note:** This will remove everything to do with the starling cluster. The dashboard access token will be deleted. The container images will remain on your machine, but to remove those as well run `docker system prune --volumes`. 

### Accessing logs on the dashboard

Please see the [instructions here](/details/kubernetes-dashboard)

### Scaling the number of drones

#### Dashboard

The simplest way to scale the number of drones is via the dashboard as shown below:

![starling-scale](/img/starling-scale-drones.gif)

The steps are as follows:

1. Select the 'Stateful Sets' page on the sidebar
2. There should be an entry called 'starling-px4-sitl' representing the drone deployment. On the right hand side, there is an option button: ☰. Press the option button and select *Scale*
3. In the *Desired replicas* field, input the number of drones you wish to deploy. Then click *Save* to deploy. 
4. Going to gazebo, the drones should start appearing. The default drone arrangement is in a discrete spiral from (0, 0). 

> **Note:** Right now, drones cannot be removed or despawned from the simulator. (Gazebo ROS2 despawning issues). If you have spawned too many, it is currently recommended that you restart gazebo and all the drones using `./script/start_px4sitl_gazebo.sh -r`.  

#### Configuration file

A more permanent way is to specify in the cluster deployment configuration file. Please see [this guide to kubernetes configuraiton files](../details/kubernetes.md#configuration-files) for more information. 

Within a kubernetes configuration of kind *Deployment* or *StatefulSet* specifcation you can set the 'replicas' option. The example configurations are all in the deployment folder of the repository. 

```yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  ...
spec:
  replicas: 3
  ...
```

## Controlling the Drones
### Offboard Control
There are two supported methods for offboard control of either the SITL or real drones.

1. Control drone directly via Mavlink, by Ground Control Station (GCS) or other Mavlink compatible method (e.g. Dronekit).
2. Control drone via ROS2 node

#### 1. Connecting a Ground Control Station via Mavlink

If a mavros or sitl instance is running, there will be a GCS link on `udp://localhost:14553` (hopefully). This means that you can run a GCS such as QGroundControl or Mission Planner:

- Create a comms link to `localhost:14553` on UDP 
- The GCS should auto detect the drone(s) 
- You should be able to control and monitor any sitl drone through the standard mavlink interface. 

This is a quick an easy way to control the SITL instance via Mavlink.

#### 2. Running Example ROS2 Offboard Controller node

An example offboard ROS2 controller can then be connected to SITL by running the following in a terminal:

```bash
./scripts/start_example_controller.sh
```

This will first build the example controller so it is available locally. Then deploy the example controller to the cluster. It will take a few minutes to startup.

When run, the example will confirm in the terminal that it has connected and that it is waiting for mission start. To start the mission, press the green go button in the [starling user interface](http://localhost:3000/html/main.html) which will send a message over `/mission_start` topic. A confirmation message should appear in the terminal, and the drone will arm (drone propellors spinning up) and takeoff. It will fly in circles for 10 seconds before landing and disarming. 

Once the controller has completed, the process will exit and the controller will restart, allowing you to repeat the controller.

If used with multiple vehicles, it will automatically find all drones broadcasting mavros topics, and start a controller for each one. 

To remove or restart the controller, use the `-d` or `-r` options respectively with the script. 

```bash
./scripts/start_example_controller.sh -d # Delete or Remove controller
./scripts/start_example_controller.sh -r # Restart controller
```

> See [example python controller](../guide/example-controller.md) for more details.

### Onboard Control
{% include 'snippets/onboard-control.md' %}

## Development on the Drone and Simulator 

### Useful Scripts:

There are a number of useful scripts in the `/scripts` directory of this repository. Scripts can be run from any location, but for this tutorial we assume the user is in the root directory.

1. `./scripts/start_k3s.sh` - This starts the cluster 
    -  `-u` will uninstall the cluster from the machine (and remove any running processes)
2. `./scripts/start_single_px4sitl_gazebo.sh` - This starts the starling core functions (Dashboard and UI). It also starts the gazebo simulator and a 'drone' process running the PX4 SITL and a connected Mavros ROS2 node. Assumes the cluster has been started.
    - `-d` will stop the gazebo and all connected 'drone' processes only (use if reloading the controller).
    - `-r` will restart the gazebo and all connected 'drone' processes. 
  labels:
    app: nginx
    - `-sk` will skip the starting/check that the starling core functions are running.
    - `-ow` will automatically open the UI webpages.
3. `./scripts/start_starling_base.sh` - This starts the starling user interface and dashboard. Automatically run by `start_single_px4sitl_gazebo.sh` 
4. `./scripts/start_dashboard.sh` - This starts the dashboard process on the cluster. Assumes the cluster has already been started. Automatically run by `start_single_px4sitl_gazebo.sh` and `start_starling_base.sh` 
    - `-ow` will automatically open the UI webpages.

> **Note:** The `./run_k3s` script internally runs `start_k3s.sh` and `start_single_px4sitl_gazebo.sh`. Any options passed will be forwarded to the relevant script.
### Modifying the example controller
In the [controllers](https://github.com/UoBFlightLab/ProjectStarling/tree/master/controllers) folder there is an example_controller_python which you should have seen in action in the example above. The ROS2 package is in [example_controller_python](https://github.com/UoBFlightLab/ProjectStarling/tree/master/controllers/example_controller_python/example_controller_python). Any edits made to the ROS2 package can be built by running `make` in the controllers directory. This will use colcon build to build the node and output a local image named `example_controller_python`. 

Inside the controllers folder, there is an annotated kubernetes config file [`k8.example_controller_python.amd64.yaml`](https://github.com/UoBFlightLab/ProjectStarling/blob/master/controllers/example_controller_python/k8.example_controller_python.amd64.yaml). This specifies the deployment of a *pod* which contains your local `example_controller_python` image ([this line](https://github.com/UoBFlightLab/ProjectStarling/blob/227b22fbd97ac58f5b34e4154e58d01f72f29988/controllers/example_controller_python/k8.example_controller_python.amd64.yaml#L49)). 

Similar to before you can start up the local example controller by using the script:
```bash
./scripts/start_example_controller.sh
```
But if you have made a copy, or wish to run your own version of the configuration, you can manually deploy your controller by running the following:
```bash
sudo k3s apply -f k8.example_controller_python.amd64.yaml
sudo k3s delete -f k8.example_controller_python.amd64.yaml # To delete
```
See [kuberenets configuration](../details/kubernetes.md) for more details.  

For debugging, you can both see the logs, and execute on your controller container through the dashboard. See [instructions here](../details/kubernetes-dashboard.md)

Inside you can `source install/setup.bash` and run ROS2 commands like normal. 

### Creating your own from scratch

Of course you can create your own controller from scratch. Inside your controller repository, the following is required
1. Your ROS2 package folder (what would usually go inside the `dev_ws/src` directory)
2. A Dockerfile (named `Dockerfile`) which is dervied `FROM uobflightlabstarling/starling-controller-base`, use the [example Dockerfile](controllers/example_controller_python/Dockerfile) as a template. 
3. A Kubernetes YAML config file specifying either a Pod or a deployment. Use the example `k8.example_controller_python.amd64.yaml` as a template. There are annoated comments. Also see [here](../details/kubernetes.md) for more details.  

Your Dockerfile can be built by running the following in the directory with the Dockerfile.
```
docker build -t <name of your controller> .
```
Once built, the configuration file must have a container config specifying your own image name. 

Your container can then be deployed manually using `k3s apply -f <your config>.yaml` as above.

## Troubleshooting/ FAQs