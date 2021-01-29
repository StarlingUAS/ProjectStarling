# Testing Mavros, Ros2, Ardupilot-sitl, Docker and Kubernetes

This is wip and contains several tests.

## Testing Mavros and Ardupilot

Firstly build Mavros and the Controller within the current ros2_talker_listener workspace:
```bash
make
```

This will build the mavros dockerfile and name it `mavros`, it will then tag this build as `starling-mavros:latest` (It will also build the controller named `controller`, tagged as `starling-controller:latest`)

### Running in separate terminals
We first test whether mavros and ardupilot can connect in their own terminals before using docker-compose. We use Arthur's sitl docker [link](https://github.com/arthurrichards77/ardupilot_sitl_docker).

```bash
docker run arthurrichards77/ardupilot-sitl-docker:latest
```
Validate it works by connecting up QGroundControl to `udp:127.0.0.1:14553` and the sitl instance should be found.

Then you can run the mavros container in a separate terminal:
```bash
docker run -it --rm --name mvr -e MAVROS_FCU_URL=<fcu_utl> mavros
```
- The environment variable should be `tcp://<ip-address-of-sitl-container>:5762`, so in most cases `tcp://172.17.0.2:5762`. 
- Can experiment with `--network="host` for mavros, should not make a difference as sitl container IP is resolved identically.
- This will `roslaunch mavros apm.launch` and then start the dynamic bridge with the option of bridging all 1to2 topics (exposing all mavros topics to ros2) [TODO only expose those that are neccessary]. See CMD in [Dockerfile.mavros](../../controller/Dockerfile.mavros)

Open another terminal and attach a shell to the mavros container to check how ros is working:
```bash
make attach_mavros_shell
#or
docker exec -it mvr bash
#or to check without ros1/2 bridge on the ros1 side
docker exec -it mvr /bin/bash -c 'source /opt/ros/noetic/setup.bash && rostopic echo /diagnostics'
#or to check with ros1/2 bridge on the ros2 side
docker exec -it mvr /bin/bash -c 'source /opt/ros/foxy/setup.bash ros2 topic echo /diagnostics'
```

### Running in Docker-Compose
There is a docker-compose file `docker-compose.mavros.yaml` which should work, but the `MAVROS_FCU_URL` is hardcoded so it may or may not line up
```bash
docker-compose -f docker-compose.mavros.yaml up 
```

## Testing Ros2 listener and talker nodes, and live workspace

See makefile for exact commands. Running `make dev_controller` will bind mount `workspace` into the `ctrl_user` home directory and give you a shell within the container. Then any `colcon build` or modifications to code within is reflected inside and outside the container. 