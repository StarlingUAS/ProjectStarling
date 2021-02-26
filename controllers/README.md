# Example Python Controller

This serves as an example Python implementation of a controller for PX4. It
begins sending offboard setpoints, sets the mode to `OFFBOARD`, arms the
vehicle, and finally begins moving the setpoint around the sky.

## Running
### Docker

To run the controller, it needs to be connected to the network that the other ROS nodes exist in.

For example:
```bash
docker run -it --network projectstarling_default example_controller_python
```
where `projectstarling_default` is the name of the default network created by the example `docker-compose.yaml` in the root directory.

Replace `example_controller_python` with `uobflightlabstarling/example_controller_python` if you have not locally built the controller. 

### Kubernetes

To run the controller in kubernetes, k3s must be up and running, then simply apply the k8 deployment script
```bash
kubectl apply -f example_controller_python/k8.example_controller_python.amd64.yaml
```

This will firstly run the local `uobflightlabstarling/example_controller_python` if one exists, otherwise it will pull from docker hub. 

## TODO

- [ ] Get it working for the clover model
- [x] Add support for a land command or similar
- [x] Add support for an emergency stop

## Notes
Due to the way rclpy works, this container can not be killed gracefully unless
using the `-it` flags for docker run. See [this issue](https://github.com/ros2/rclpy/issues/527).
