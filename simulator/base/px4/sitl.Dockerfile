# syntax = edrevo/dockerfile-plus

# Build *only* the PX4 SITL

# Include the common Dockerfile to get the PX4 repo
INCLUDE+ ./px4builder.Dockerfile

# Add support for remote host in SITL & build it
RUN make px4_sitl

FROM ros:foxy-ros-base-focal

# Copy built PX4 repo into this image
COPY --from=px4builder /src /src

ENV PX4_INSTANCE 0
ENV PX4_INSTANCE_BASE 0
ENV PX4_SIM_IP 127.0.0.1
ENV PX4_OFFBOARD_IP 127.0.0.1
ENV PX4_OFFBOARD_PORT_BASE 14830
ENV PX4_BUILD_PATH /src/PX4-Autopilot/build/
ENV PX4_SIM_MODEL iris
ENV PX4_SIM_INIT_LOC_X=0 PX4_SIM_INIT_LOC_Y=0 PX4_SIM_INIT_LOC_Z=0
ENV PX4_SIM_FORCE_USE_SET_POSITION false
ENV SIM_WD /sim_wd

# Modify startup script to use environment for server IP
RUN sed -i 's/simulator start -c $simulator_tcp_port/simulator start -t $PX4_SIM_IP $simulator_tcp_port/' /src/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/px4-rc.simulator

# Modify startup script to add partner IP for offboard script
RUN sed -i 's/\(mavlink start -x -u $udp_offboard_port_local -r 4000000 -f -m onboard -o $udp_offboard_port_remote\)/\1 -t $PX4_OFFBOARD_IP -p/' /src/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/px4-rc.mavlink
RUN sed -i '/udp_onboard_payload_port_local/d' /src/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/px4-rc.mavlink
RUN sed -i '/udp_onboard_gimbal_port_local/d' /src/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/px4-rc.mavlink

# Modify startup script to enable mavlink broadcasting
RUN sed -i '/param set IMU_INTEG_RATE 250/a param set MAV_${px4_instance}_BROADCAST 1' /src/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/rcS

# Modify startup script to change range of udp offboard remote connection ports (and remove line for if px4_instance > 9)
RUN sed -i 's/udp_offboard_port_remote=$((14540+px4_instance))/udp_offboard_port_remote=$((PX4_OFFBOARD_PORT_BASE+px4_instance))/'  /src/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/px4-rc.mavlink
RUN sed -i '/[ $px4_instance -gt 9 ] && udp_offboard_port_remote=14549 # use the same ports for more than 10 instances to avoid port overlaps/d' /src/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/px4-rc.mavlink

# Add entrypoint to handle PX4_SIM_HOST instead of IP
COPY entrypoint.sh /entrypoint.sh
ENTRYPOINT [ "/entrypoint.sh" ]

# Add a volume for the SITL eeprom
VOLUME ${SIM_WD}

# Relies on remote host option for simulator
CMD /src/PX4-Autopilot/build/px4_sitl_default/bin/px4 \
      -d -i ${PX4_INSTANCE} -s etc/init.d-posix/rcS \
      -w /sim_wd \
      /src/PX4-Autopilot/ROMFS/px4fmu_common