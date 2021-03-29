# syntax = edrevo/dockerfile-plus
ARG VERSION=latest

INCLUDE+ ./px4builder.Dockerfile

# Build the gazebo link
RUN make px4_sitl sitl_gazebo

FROM uobflightlabstarling/starling-sim-base-core:${VERSION}

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
