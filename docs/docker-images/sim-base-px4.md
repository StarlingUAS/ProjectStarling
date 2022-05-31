# `starling-sim-base-px4`

Based on [`starling-sim-base-core`](../sim-base-core)

This image contains the base Gazebo installation and adds the PX4 specific plugins.

## Environment Variables

Name                  | Default Value                | Description
----------------------|------------------------------|------------
`PX4_SIM_HOST`      |                | The host address of the simulation, if set, it will look up the ip address and assign to `PX4_SIM_IP`
`PX4_OFFBOARD_HOST` | | The host adress of the offboard (mavros), if set, it will look up the ip address and assign to `PX4_OFFBOARD_IP`
`PX4_INSTANCE`          | 0                            | The instance of the SITL (set to PX4 SYS_ID set to `PX4_INSTACE+1`), can be set to `ordinal` to automatically derive from last number of hostname (e.g. hostname-4), or set to 0 < instace < 254.
`PX4_HOME_LAT`          | 51.501582                    | Home Latitude
`PX4_HOME_LON`          | -2.551791                    | Home Longitude
`PX4_HOME_ALT`          | 0                            | Home Altitude
`PX4_SIM_MODEL`         | iris                         | The PX4 simulation model which matches the available PX4 model library (see `KEEP_PX4_VEHICLES`)
`PX4_SIM_INIT_LOC_X`    | 0                            | Virtual X location for vehicle to spawn
`PX4_SIM_INIT_LOC_Y`    | 0                            | Virtual Y location for vehicle to spawn
`PX4_SIM_INIT_LOC_Z`    | 0                            | Virtual Z location for vehicle to spawn
`PX4_SIM_FORCE_USE_SET_POSITION` | false               | If multiple vehicles are spawning, by default they will spawn in a spiral. This forces the use of Init Locations
`PX4_SYSID_SITL_BASE`   | 200                          | The base value of the minimum STIL instances actual instance is `PX4_SYSID_SITL_BASE` + `PX4_INSTACE` (not implemented yet)
`ENABLE_EXTERNAL_VISION` |                             | If this variable exists, sets set px4 params `EKF2_HGT_MODE` to 3 and `EKF2_AID_MASK` to 24 (EV_POS+EV_YAW) for use with a simulated vicon system.
`KEEP_PX4_VEHICLES` | ""                | [**BUILD ARG**] A string of the format `! -path ./<vehicle_1> ! -path ./<vehicle_2>`... which specifies a list of extra vehicle PX4 models to keep on top of `sun`, `gps`, `iris` and `r1_rover`.
`REMOVE_GAZEBO_LIBRARIES` | "libgazebo_video_stream_widget.so libgazebo_airspeed_plugin.so libgazebo_drop_plugin.so libgazebo_user_camera_plugin.so libgazebo_camera_manager_plugin.so libgazebo_gimbal_controller_plugin.so libgazebo_opticalflow_mockup_plugin.so libgazebo_opticalflow_plugin.so libgazebo_airship_dynamics_plugin.so libgazebo_catapult_plugin.so libgazebo_irlock_plugin.so libgazebo_parachute_plugin.so libgazebo_sonar_plugin.so libgazebo_usv_dynamics_plugin.so libgazebo_uuv_plugin.so libgazebo_wind_plugin.so" | [**BUILD ARG**] A space seperated list of libraries to be removed from the image. See below for details.


### `REMOVE_GAZEBO_LIBRARIES`

Default list of available plugins which are built by the PX4_SITL:
```
libgazebo_airship_dynamics_plugin.so X
libgazebo_airspeed_plugin.so X
libgazebo_barometer_plugin.so
libgazebo_camera_manager_plugin.so X
libgazebo_catapult_plugin.so X
libgazebo_controller_interface.so
libgazebo_drop_plugin.so X
libgazebo_gimbal_controller_plugin.so X
libgazebo_gps_plugin.so
libgazebo_groundtruth_plugin.so
libgazebo_gst_camera_plugin.so
libgazebo_imu_plugin.so
libgazebo_irlock_plugin.so X
libgazebo_lidar_plugin.so
libgazebo_magnetometer_plugin.so
libgazebo_mavlink_interface.so
libgazebo_motor_model.so
libgazebo_multirotor_base_plugin.so
libgazebo_opticalflow_mockup_plugin.so X
libgazebo_opticalflow_plugin.so X
libgazebo_parachute_plugin.so X
libgazebo_sonar_plugin.so X
libgazebo_user_camera_plugin.so X
libgazebo_usv_dynamics_plugin.so X
libgazebo_uuv_plugin.so X
libgazebo_video_stream_widget.so X
libgazebo_vision_plugin.so
```
A space seperated list of these files can be given at build time to remove all of the plugins you dont need in order to save space. Each library is around 7 to 9Mb.