# Rosbridge-suite

Builds a container that serves a ros2 web-bridge using the [rosbridge-suite](https://github.com/RobotWebTools/rosbridge_suite)

The rosbridge is known as a 'json' bridge as it converts topics into json to be sent via websocket.

## Installation

This is built using the `ui` target in the system `Makefile` to the container `uobflightlabstarling/rosbridge-suite`:
```
cd <root of ProjectStarling>
make ui
```

## Running

This bridge is run using with default port `9090`:

*Linux*:
```
docker run -it --rm --net=host uobflightlabstarling/rosbridge-suite
```

*Windows*:
```
docker run -it --rm --network=projectstarling_default -p 9090:9090 uobflightlabstarling/rosbridge-suite
```

The port can be specified by passing the environment variable `ROSBRIDGE_PORT`, i.e.
```
docker run -it --rm --net=host -e ROSBRIDGE_PORT=<port> -p <localport>:<port> uobflightlabstarling/rosbridge-suite
```

## External ROS messages

Core rosbridge suite currently only supports `mavros_msgs` and `gazebo_msgs` ontop of the standard ROS2 messages.

However, external messages can be built at runtime by bind mounting your messages into the source folder of the messages workspace defined by environment variable `MSGS_WS`.

For example if you had local msgs `controller_msgs`:
```console
docker run -it --rm --network px4_default -p 9090:9090 -v "$(pwd)"/controller_msgs:/msgs_ws/src/controller_msgs -e MSGS_WS=/msgs_ws uobflightlabstarling/rosbridge-suite:latest
```

or equivalently in a docker-compose file:
```yaml
services:
  rosbridge-suite:
    image: uobflightlabstarling/rosbridge-suite:latest
    environment:
      - "MSGS_WS=/msgs_ws"
    volumes:
      - ./controller_msgs:/msgs_ws/src/controller_msgs
    ports:
      - "9090:9090"
```

Note that this would work best for small numbers of extra message topics. If you have a large number consider writing your own dockerfile which uses this one.

## Application

This can be used with any application which supports the ros2 web bridge suite.

An example ui application is given in [StarlingUAS/starling_ui_example](https://github.com/StarlingUAS/starling_ui_example)

A more complex system would be the dashboard created by [foxglove studios](https://foxglove.dev/): https://studio.foxglove.dev/?

> Note: Starling doesn't quite support foxglove just yet, there is a known bug here: https://github.com/foxglove/studio/issues/2035. If this is fixed it will be compatible.