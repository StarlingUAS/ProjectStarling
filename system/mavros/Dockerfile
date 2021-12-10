FROM ros:foxy-ros-base-focal

# Install mavros
RUN apt-get update \
    && apt-get install --no-install-recommends -y \
      ros-${ROS_DISTRO}-mavros \
    && rm -rf /var/lib/apt/lists/

# Part of MAVROS setup
RUN /opt/ros/${ROS_DISTRO}/lib/mavros/install_geographiclib_datasets.sh

COPY ros_entrypoint.sh /ros_entrypoint.sh

WORKDIR /ros_ws

COPY mavros_setup.sh .

COPY mavros.launch.xml /ros_ws/launch/mavros.launch.xml
COPY config/mavros_config_*.yaml /
COPY config/mavros_pluginlists_*.yaml /

ENV MAVROS_MOD_CONFIG_PATH="/mavros_config_mod.yaml"
ENV MAVROS_CONFIG_PATH="/mavros_config_px4.yaml"
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
