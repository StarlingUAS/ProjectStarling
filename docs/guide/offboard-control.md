There are two supported methods for offboard control of either the SITL or real drones.

1. Control drone directly via Mavlink, by Ground Control Station (GCS) or other Mavlink compatible method (e.g. Dronekit).
2. Control drone via ROS2 node

### 1. Connecting a Ground Control Station via Mavlink

If a mavros or sitl instance is running, there will be a GCS link on `udp://localhost:14553` (hopefully). This means that you can run a GCS such as QGroundControl or Mission Planner:

- Create a comms link to `localhost:14553` on UDP 
- The GCS should auto detect the drone(s) 
- You should be able to control and monitor any sitl drone through the standard mavlink interface. 

This is a quick an easy way to control the SITL instance via Mavlink.

### 2. Running Example ROS2 Offboard Controller node

An example offboard ROS2 controller can then be conncted to SITL by running the following in a separate terminal:

```bash
docker run -it --rm --network projectstarling_default uobflightlabstarling/example_controller_python
```
This will download and run the `example_controller_python` image from `uobflightlabstarling` on docker hub. `-it` opens an interactive terminal. `--rm` removes the container when completed. `--network` attaches the container to the default network created by `make run` or `docker-compose`. The default network name is `<foldername>_default`.

> **Note:** The controller may complain that it cannot find the drone. Double check that the name of the root folder matches the one passed to `--network`.

When run, the example will confirm in the terminal that it has connected and that it is waiting for mission start. To start the mission, press the green go button in the [starling user interface](http://localhost:3000/html/main.html) which will send a message over `/mission_start` topic. A confirmation message should appear in the terminal, and the drone will arm (drone propellors spinning up) and takeoff. It will fly in circles for 10 seconds before landing and disarming. 

If used with multiple vehicles, it will automatically find all drones broadcasting mavros topics, and start a controller for each one. 
