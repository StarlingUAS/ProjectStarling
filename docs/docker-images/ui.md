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

## Application

This can be used with any application which supports the ros2 web bridge suite. 

An example ui application is given in [StarlingUAS/starling_ui_example](https://github.com/StarlingUAS/starling_ui_example)

A more complex system would be the dashboard created by [foxglove studios](https://foxglove.dev/): https://studio.foxglove.dev/?

> Note: Starling doesn't quite support foxglove just yet, there is a known bug here: https://github.com/foxglove/studio/issues/2035. If this is fixed it will be compatible. 

### Kubernetes Deployment

The UI can also be run within the kubernetes deployment and network. To start as a kubernetes Deployment simply apply the `kubernetes.yaml` file in this directory.
```bash
sudo k3s kubectl apply -f kubernetes.yaml
```

## Notes

This dockerfile installs mavros-foxy and gazebo-foxy messages. 

If other messages are required then either this container needs to be rebuilt with the new messages included (e.g. a build step) or they need to be injected in using the `ros.env.d` pipeline.