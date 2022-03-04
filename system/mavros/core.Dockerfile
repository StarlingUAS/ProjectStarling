FROM ros:foxy-ros-base-focal AS builder

WORKDIR /ros_ws

# Install mavros dependencies
COPY mavros/libmavconn/package.xml /ros_ws/src/mavros/libmavconn/package.xml
COPY mavros/mavros/package.xml /ros_ws/src/mavros/mavros/package.xml
COPY mavros/mavros_msgs/package.xml /ros_ws/src/mavros/mavros_msgs/package.xml
COPY mavros/mavros_extras/package.xml /ros_ws/src/mavros/mavros_extras/package.xml

RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && apt-get update \
    && rosdep install -y --from-paths src --ignore-src .

# Install mavros itself
COPY mavros /ros_ws/src/mavros

RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && colcon build

RUN /ros_ws/install/mavros/lib/mavros/install_geographiclib_datasets.sh

# Start the main image build
FROM ros:foxy-ros-core-focal AS image

WORKDIR /ros_ws

# Copy over the build artifacts
COPY --from=builder /ros_ws/install /ros_ws/install

# Install exec dependencies
RUN rm install/COLCON_IGNORE \
    && apt-get update \
    && apt-get install -y --no-install-recommends python3-rosdep \
    && rosdep init \
    && rosdep update --rosdistro $ROS_DISTRO \
    && . /opt/ros/${ROS_DISTRO}/setup.sh \
    && rosdep install -t exec -y --from-paths install --ignore-src \
    && apt-get remove -y python3-rosdep \
    && SUDO_FORCE_REMOVE=yes apt-get autoremove -y \
    && rm -rf /var/lib/apt/lists/*

# Part of MAVROS setup
COPY --from=builder /usr/share/GeographicLib /usr/share/GeographicLib

COPY ros_entrypoint.sh /ros_entrypoint.sh

COPY mavros_setup.sh .

COPY mavros.launch.xml /ros_ws/launch/mavros.launch.xml
COPY config/mavros_config_*.yaml /
COPY config/mavros_pluginlists_*.yaml /

ENV MAVROS_MOD_CONFIG_PATH="/mavros_config_mod.yaml"
ENV MAVROS_CONFIG_PATH="/mavros_config_px4.yaml"
ENV MAVROS_MOD_PLUGINLISTS_PATH="/mavros_pluginlists_mod.yaml"
ENV MAVROS_PLUGINLISTS_PATH="/mavros_pluginlists_px4.yaml"

# Add custom ROS DDS configuration (force UDP always)
COPY fastrtps_profiles.xml /ros_ws/
ENV FASTRTPS_DEFAULT_PROFILES_FILE /ros_ws/fastrtps_profiles.xml

# TODO: Rename to FCU_PROTOCOL
ENV MAVROS_FCU_CONN="udp"
# TODO: Rename to FCU_HOST
ENV MAVROS_FCU_IP="127.0.0.1"
# TODO: Rename to PORT_BASE
ENV MAVROS_FCU_UDP_BASE="14830"
ENV MAVROS_TGT_SYSTEM="auto"
ENV PX4_INSTANCE_BASE=0
ENV MAVROS_TGT_FIRMWARE="px4"
ENV MAVROS_GCS_URL="udp-pb://@:14550"

CMD [ "ros2", "launch", "launch/mavros.launch.xml"]
