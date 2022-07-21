FROM ros:foxy-ros1-bridge

# Install mavros
RUN apt-get update \
    && apt-get install --no-install-recommends -y \
      ros-${ROS1_DISTRO}-mavros \
      ros-${ROS1_DISTRO}-mavros-extras \
      ros-${ROS2_DISTRO}-mavros-msgs \
      ros-foxy-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/

# Part of MAVROS setup
RUN /opt/ros/${ROS1_DISTRO}/lib/mavros/install_geographiclib_datasets.sh

# We then need to build the bridge from source
#  (So we probably don't need to use the ros1-bridge image...)
# Remove the preinstalled version
RUN apt-get remove -y ros-foxy-ros1-bridge
# Get the source (use specific commit hash instead of foxy due to this issue when building mavros: https://github.com/ros2/ros1_bridge/issues/313)
RUN git clone -b foxy https://github.com/ros2/ros1_bridge /bridge_ws/src/ros1_bridge \
    && cd /bridge_ws/src/ros1_bridge \
    && git checkout 82d1f4588761d6e54eeaca821398137be0e475f1

# Swap line 36 and 41 to fix this issue: https://github.com/ros2/ros1_bridge/issues/316
# For use of parameter server.
RUN sed -i "41d" /bridge_ws/src/ros1_bridge/src/parameter_bridge.cpp \
    && sed -i "36 i rclcpp::init(argc, argv);" /bridge_ws/src/ros1_bridge/src/parameter_bridge.cpp

# Build with ROS2 mavros_msgs on path
RUN cd /bridge_ws \
    && . /opt/ros/${ROS1_DISTRO}/setup.sh \
    && . /opt/ros/${ROS2_DISTRO}/setup.sh \
    && export CMAKE_PREFIX_PATH=$AMENT_PREFIX_PATH:$CMAKE_PREFIX_PATH \
    && colcon build --packages-select ros1_bridge --cmake-force-configure \
    && rm -r build

RUN mkdir /ros_ws

COPY mavros_bridge.launch.xml /ros_ws/launch/
COPY run_ros1.sh /ros_ws/scripts/

COPY ros_entrypoint.sh /ros_entrypoint.sh

WORKDIR /ros_ws

COPY mavros_setup.sh .

COPY mavros.launch /ros_ws/launch/mavros.launch
COPY config/mavros_config_*.yaml /
COPY config/mavros_pluginlists_*.yaml /
COPY config/bridge_config.yaml /

ENV MAVROS_MOD_CONFIG_PATH="/mavros_config_mod.yaml"
ENV MAVROS_CONFIG_PATH="/mavros_config_px4.yaml"
ENV MAVROS_PLUGINLISTS_PATH="/mavros_pluginlists_px4.yaml"

ENV BRIDGE_MOD_CONFIG_PATH="/bridge_config_mod.yaml"
ENV BRIDGE_CONFIG_PATH="/bridge_config.yaml"

# Build extra custom nodes in Mavros
COPY starling_mavros_extras /ros_ws/src/starling_mavros_extras
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && export CMAKE_PREFIX_PATH=$AMENT_PREFIX_PATH:$CMAKE_PREFIX_PATH \
    && cd /ros_ws \
    && colcon build --packages-select starling_mavros_extras \
    && rm -r build

# Add custom ROS DDS configuration (force UDP always)
COPY cyclone_profile.xml /ros_ws/
ENV CYCLONEDDS_URI /ros_ws/cyclone_profile.xml

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

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

CMD [ "ros2", "launch", "launch/mavros_bridge.launch.xml"]
