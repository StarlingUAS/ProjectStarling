# Example Python Controller

This serves as an example Python implementation of a controller for PX4. It
begins sending offboard setpoints, sets the mode to `OFFBOARD`, arms the
vehicle, and finally begins moving the setpoint around the sky.

## Running

To run the controller, it needs to be connected to the network that the other ROS nodes exist in.
For example:
```bash
docker run -it --network starling_default example_controller_python
```

## TODO

- [ ] Get it working for the clover model
- [ ] Add support for a land command or similar
- [ ] Add support for an emergency stop

## Notes
Due to the way rclpy works, this container can not be killed gracefully unless
using the `-it` flags for docker run. See [this issue](https://github.com/ros2/rclpy/issues/527).
