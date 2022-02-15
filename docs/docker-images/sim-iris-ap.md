# `starling-sim-iris-ap`

Based on [`staring-sim-ardupilot-gazebo`](../sim-ardupilot-gazebo)
See for additional environment variables.

## Overview

This image adds the launch files required to launch an Iris model with the ArduPilot plugin using Gazebo.

The [iris ardupilot model](https://github.com/khancyr/ardupilot_gazebo/tree/master/models/iris_with_ardupilot) depends on the iris with standoffs and a *gimbal_small_2d*. 

The original gimbal in the gazebo model library is out of date, therefore an updated one is included in this image. It also includes an updated plugin which allows control of the gimbal and streams the camera image. 
## Environment Variables

Name                  | Default Value                | Description
----------------------|------------------------------|------------
`CAMERA_NAME`         | camera                       | ROS2 Name of the camera, camera topic is `$VEHICLE_NAMESPACE/$CAMERA_NAME/image_raw`. 
`CAMERA_HEIGHT`       | 480                          | Height resolution of camera image
`CAMERA_WIDTH`        | 640                          | Width resolution of camera image
`GIMBAL_INITIAL_ANGLE`| 0.785                        | Initial angle (radians) of the gimbal. 0.0 Angle is forwards, pi/2 is down. 

## Exposed Topics

Name                  | Topic                | Description
----------------------|------------------------------|------------
`$VEHICLE_NAMESPACE/$CAMERA_NAME/image_raw` | `sensor_msgs/msg/Image` | Image topic from the camera attached to the gimbal
`$VEHICLE_NAMESPACE/$CAMERA_NAME/camera_info` | `sensor_msgs/msg/CameraInfo` | Camera Info topic from the camera attached to the gimbal
`$VEHICLE_NAMESPACE/gimbal_tilt_cmd` | `std_msgs/msg/Float32` | The target angle (radians) of the gimbal camera tilt [0.0, 3.14]. 
`$VEHICLE_NAMESPACE/gimbal_tilt_status` | `std_msgs/msg/Float32` | The current angle (radians) of the gimbal camera tilt [0.0, 3.14]. 