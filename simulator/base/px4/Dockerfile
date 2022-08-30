# syntax = edrevo/dockerfile-plus

# Add the PX4 plugins to the Gazebo image
# Produces an image based on the core gazebo image that includes the PX4-specific plugins

ARG VERSION=latest
ARG REGISTRY

# Include the common Dockerfile to get the PX4 repo
INCLUDE+ ./px4builder.Dockerfile

# Build the gazebo link
RUN make px4_sitl sitl_gazebo

# See below, needs to be of format '! -path ./<vehicle_1> ! -path ./<vehicle_2>' etc
ARG KEEP_PX4_VEHICLES=""

# Default list of available plugins which are built.
#libgazebo_airship_dynamics_plugin.so X
#libgazebo_airspeed_plugin.so X
#libgazebo_barometer_plugin.so
#libgazebo_camera_manager_plugin.so X
#libgazebo_catapult_plugin.so X
#libgazebo_controller_interface.so
#libgazebo_drop_plugin.so X
#libgazebo_gimbal_controller_plugin.so X
#libgazebo_gps_plugin.so
#libgazebo_groundtruth_plugin.so
#libgazebo_gst_camera_plugin.so
#libgazebo_imu_plugin.so
#libgazebo_irlock_plugin.so X
#libgazebo_lidar_plugin.so
#libgazebo_magnetometer_plugin.so
#libgazebo_mavlink_interface.so
#libgazebo_motor_model.so
#libgazebo_multirotor_base_plugin.so
#libgazebo_opticalflow_mockup_plugin.so X
#libgazebo_opticalflow_plugin.so X
#libgazebo_parachute_plugin.so X
#libgazebo_sonar_plugin.so X
#libgazebo_user_camera_plugin.so X
#libgazebo_usv_dynamics_plugin.so X
#libgazebo_uuv_plugin.so X
#libgazebo_video_stream_widget.so X
#libgazebo_vision_plugin.so
#libgazebo_wind_plugin.so X
ARG REMOVE_GAZEBO_LIBRARIES="libgazebo_video_stream_widget.so libgazebo_airspeed_plugin.so libgazebo_drop_plugin.so libgazebo_user_camera_plugin.so libgazebo_camera_manager_plugin.so libgazebo_gimbal_controller_plugin.so libgazebo_opticalflow_mockup_plugin.so libgazebo_opticalflow_plugin.so libgazebo_airship_dynamics_plugin.so libgazebo_catapult_plugin.so libgazebo_irlock_plugin.so libgazebo_parachute_plugin.so libgazebo_sonar_plugin.so libgazebo_usv_dynamics_plugin.so libgazebo_uuv_plugin.so libgazebo_wind_plugin.so"

# Remove a bunch of stuff before copying over
# Remove other simulators
WORKDIR /src/PX4-Autopilot/Tools
RUN rm -r flightgear_bridge jMAVSim jsbsim_bridge

# Remove unused default models
# See the list here: https://github.com/PX4/PX4-SITL_gazebo/tree/25138e803ee8525ee5fe4e6d511506e88e3f819c/models
# Remove everything but the ones specified in the command here.
WORKDIR /src/PX4-Autopilot/Tools/sitl_gazebo/models
RUN find . -maxdepth 1 -type d \
        ! -path . \
        ! -path ./sun \
        ! -path ./gps \
        ! -path ./iris \
        ! -path ./r1_rover  \
        ! -path ./rover  \
        ${KEEP_PX4_VEHICLES} \
        -exec rm -r '{}' \;

# Remove unused build artefacts and extra px4-gazebo plugin libraries
WORKDIR /src/PX4-Autopilot/build/px4_sitl_default/build_gazebo
RUN rm -r CMakeFiles ${REMOVE_GAZEBO_LIBRARIES} *.cc *.h *.ninja unit_tests include

FROM ${REGISTRY}uobflightlabstarling/starling-sim-base-core:${VERSION}

# Copy built PX4 repo into this image
COPY --from=px4builder /src/PX4-Autopilot/build/px4_sitl_default/build_gazebo /src/PX4-Autopilot/build/px4_sitl_default/build_gazebo
COPY --from=px4builder /src/PX4-Autopilot/Tools /src/PX4-Autopilot/Tools/


# Add PX4/Gazebo environment setup to a ros.env that is sourced by entrypoint
RUN echo "source /src/PX4-Autopilot/Tools/setup_gazebo.bash /src/PX4-Autopilot /src/PX4-Autopilot/build/px4_sitl_default" >> /ros.env

WORKDIR /ros_ws

# Set the gazebo origin's geographic coordinates
ENV PX4_HOME_LAT 51.501582
ENV PX4_HOME_LON -2.551791
ENV PX4_HOME_ALT 0.0
ENV PX4_SIM_MODEL iris
ENV PX4_SIM_INIT_LOC_X=0 PX4_SIM_INIT_LOC_Y=0 PX4_SIM_INIT_LOC_Z=0
ENV PX4_SIM_FORCE_USE_SET_POSITION false
ENV PX4_INSTANCE 0
ENV PX4_SYSID_SITL_BASE 200

# Add entrypoint to handle PX4_SIM_HOST instead of IP
COPY entrypoint.sh /entrypoint.sh
ENTRYPOINT [ "/entrypoint.sh" ]

CMD [ "ros2", "launch", "/ros_ws/launch/sim.launch.xml" ]
