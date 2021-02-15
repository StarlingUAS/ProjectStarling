# Example Python Controller

**Currently does not fly in control**

This serves as an example Python implementation of a controller for PX4. It
begins sending offboard setpoints, sets the mode to `OFFBOARD`, arms the
vehicle, and finally begins moving the setpoint around the sky.

## Issues

Currently, the vehicle does not fly in control for some unknown reason.


## TODO

- [ ] Actually get it working
- [ ] Add support for a land command or similar
- [ ] Add support for an emergency stop

## Notes
Due to the way rclpy works, this container can not be killed gracefully unless
using the `-it` flags for docker run. See [this issue](https://github.com/ros2/rclpy/issues/527).