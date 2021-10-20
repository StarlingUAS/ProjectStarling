# Starling Web bridge

Builds a container that serves a web bridge on 
RobotWebTools' `ros2-web-bridge` and `rclnodejs` internally.

This is built using the `ui` target in the system `Makefile` to the container `uobflightlabstarling/ros-web-bridge:
```
cd <root of ProjectStarling>
make ui
```

This bridge is run using with default port `9090`
```
docker run -it --rm --net=host -p 9090:9090 uobflightlabstarling/ros-web-bridge
```

The port can be specified by passing the environment variable `ROSBRIDGE_PORT`, i.e.
```
docker run -it --rm --net=host -e ROSBRIDGE_PORT=<port> -p <localport>:<port> uobflightlabstarling/ros-web-bridge
```



