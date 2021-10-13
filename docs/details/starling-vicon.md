# `starling-vicon`

This image contains a Vicon UDP to ROS2 bridge. It has some additional configuration to work with Starling, in that it
reads the `/etc/starling/vehicle.config` file. This listens on port 51001, (the default Vicon UDP port) and retrieves
the pose information for a target object. This pose information is then forwarded to the appropriate MAVROS topic.

## Contents
[TOC]

## Environment Variables

Name                | Default Value      | Description
--------------------|--------------------|------------
`VEHICLE_NAME`      | {unset}            | Vicon target object name.
`VEHICLE_NAMESPACE` | {unset}            | Namespace for mavros topics. Set to __override__ default value of `vehicle_${VEHICLE_MAVLINK_SYSID}`

When unset, both `VEHICLE_NAME` and `VEHICLE_NAMESPACE` are derived from the `vehicle.config` file, reading the name and
MAVLink system ID defined there.


## Behaviour Notes

### Running on a vehicle

Ensure `/etc/starling/vehicle.config` is mounted. The container is then configured from the contents of that file.
