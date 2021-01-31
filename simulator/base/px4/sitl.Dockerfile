# Use image built for the gazebo plugins
FROM starling-px4-builder as px4sitl

# Add support for remote host in SITL
RUN git fetch --recurse-submodules=no origin fe7908feb0de5ee8a4465619098f09b987ab011d \
    && git cherry-pick --no-commit fe7908feb0de5ee8a4465619098f09b987ab011d
RUN make px4_sitl

FROM ros:foxy-ros-base-focal

# Copy built PX4 repo into this image
COPY --from=px4sitl /src /src

ENV PX4_INSTANCE 0
ENV PX4_SIM_IP 127.0.0.1
ENV PX4_BUILD_PATH /src/PX4-Autopilot/build/
ENV PX4_SIM_MODEL iris
ENV SIM_WD /sim_wd

# Modify startup script to use environment for server IP
RUN sed -i 's/simulator start -c $simulator_tcp_port/simulator start -t $PX4_SIM_IP $simulator_tcp_port/' /src/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/rcS

# Modify startup script to enable mavlink broadcasting
RUN sed -i '/param set IMU_INTEG_RATE 250/a param set MAV_BROADCAST 1' /src/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/rcS

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