## Development on the Drone and Simulator

### Useful Scripts:

There are a number of useful scripts in the `/scripts` directory of this repository. Scripts can be run from any location, but for this tutorial we assume the user is in the root directory.

1. `./scripts/start_k3s.sh` - This starts the cluster
    -  `-u` will uninstall the cluster from the machine (and remove any running processes)
    - `-sk` will skip the installation check for k3s
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
k3s apply -f k8.example_controller_python.amd64.yaml
k3s delete -f k8.example_controller_python.amd64.yaml # To delete
```
See [kubernetes configuration](../details/kubernetes.md) for more details.

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

Your container can then be deployed manually using `kubectl -f <your config>.yaml` as above.
