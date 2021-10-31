# `starling-sim-iris-ap`

Based on [`staring-sim-ardupilot-gazebo`](../sim-ardupilot-gazebo)

## Overview

This image adds the launch files required to launch an Iris model with the ArduPilot plugin using Gazebo.

The [iris ardupilot model](https://github.com/khancyr/ardupilot_gazebo/tree/master/models/iris_with_ardupilot) depends on the iris with standoffs and a *gimbal_small_2d*. 

The original gimbal in the gazebo model library is out of date, therefore an updated one is included in this image. It also includes an updated plugin which allows control of the gimbal and streams the camera image. 
## Environment Variables

Name                  | Default Value                | Description
----------------------|------------------------------|------------
`AP_SITL_ADDRESS`     | 127.0.0.1                    | IP address for Gazebo plugin to use to talk to ArduPilot instance
`AP_SITL_HOST`        | {null}                       | Hostname for Gazebo plugin to use to talk to ArduPilot instance. Set to __override__ IP address.
