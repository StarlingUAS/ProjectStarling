# Initial UI Example

Builds a container that serves a web-based UI for the system. Uses
RobotWebTools' `ros2-web-bridge` and `rclnodejs` internally.

Contains an emergency stop example that publishes a `std_msgs/String` to
the topic `/emergency_stop`.

### Building

Build through Makefile as normal, target name: `ui` from the `system` Makefile


### Running

To start the image:
`docker run -it -p 3000:3000 -p 9090:9090 starling-ui`

NB: Port 3000 is exposed for the webserver, while port 9090 is exposed for the
websocket.

Once the container is running, navigate to:
[`http://127.0.0.1:3000/html/stop.html`](http://127.0.0.1:3000/html/stop.html)

In a connected ros2 instance, the command below can be run to see the output.
Note, if the topic has not yet been published to, the command will exit
immediately complaining that it could not determine the type. There are two
fixes: either press the E-STOP button to publish an initial message or supply
the type: `std_msgs/String` as a further argument.

```bash
ros2 topic echo /emergency_stop
```


### Notes

To support the full gamut of MAVROS messages, this will likely need to be built
on top of an image containing the mavros_msgs package for ROS2. Lots of things
are likely to need this so it may be worth making this one of the base layers
for the system.