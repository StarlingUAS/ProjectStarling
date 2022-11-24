FROM ros:foxy-ros-base-focal

WORKDIR /ros_ws

# Install mavros from apt
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        libboost-system-dev \
        wget \
        python3-pip \
        ros-${ROS_DISTRO}-mavros\
        ros-${ROS_DISTRO}-mavros-extras \
        ros-${ROS_DISTRO}-mavros-msgs \
        libgeographic-dev \
        geographiclib-tools \
    && rm -rf /var/lib/apt/lists/*

## Ensure geographiclib is properly installed with FindGeographicLib available
RUN wget https://raw.githubusercontent.com/mavlink/mavros/ros2/mavros/scripts/install_geographiclib_datasets.sh \
    && bash install_geographiclib_datasets.sh \
    && ln -s /usr/share/cmake/geographiclib/FindGeographicLib.cmake /usr/share/cmake-3.16/Modules/

COPY ros_entrypoint.sh /ros_entrypoint.sh

COPY mavros_setup.sh .

COPY mavros.launch.xml /ros_ws/launch/mavros.launch.xml
COPY config/mavros_config_*.yaml /
COPY config/mavros_pluginlists_*.yaml /

ENV MAVROS_MOD_CONFIG_PATH="/mavros_config_mod.yaml"
ENV MAVROS_CONFIG_PATH="/mavros_config_px4.yaml"
ENV MAVROS_MOD_PLUGINLISTS_PATH="/mavros_pluginlists_mod.yaml"
ENV MAVROS_PLUGINLISTS_PATH="/mavros_pluginlists_px4.yaml"

# Build extra custom nodes in Mavros
COPY starling_mavros_extras /ros_ws/src/starling_mavros_extras
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && export CMAKE_PREFIX_PATH=$AMENT_PREFIX_PATH:$CMAKE_PREFIX_PATH \
    && cd /ros_ws \
    && colcon build --packages-select starling_mavros_extras \
    && rm -r build

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
