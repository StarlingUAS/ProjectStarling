FROM ros:foxy-ros1-bridge

# Install mavros
RUN apt-get update \
    && apt-get install --no-install-recommends -y \
      ros-${ROS1_DISTRO}-mavros \
      ros-${ROS1_DISTRO}-mavros-extras \
    && rm -rf /var/lib/apt/lists/

# Part of MAVROS setup
RUN /opt/ros/${ROS1_DISTRO}/lib/mavros/install_geographiclib_datasets.sh

# In order for the bridge to bridge all mavros topics, we need to build a
#  ROS2 equivalent of mavros_msgs
# Clone the repo
RUN git clone -b mavros2 https://github.com/rob-clarke/mavros /ros_ws/src/mavros
# Fetch dependencies
RUN . /opt/ros/${ROS2_DISTRO}/setup.sh \
    && apt-get update \
    && rosdep install --from-paths /ros_ws/src/mavros/mavros_msgs \
    && rm -rf /var/lib/apt/lists/
# Build the package
RUN . /opt/ros/${ROS2_DISTRO}/setup.sh \
    && export CMAKE_PREFIX_PATH=$AMENT_PREFIX_PATH:$CMAKE_PREFIX_PATH \
    && cd /ros_ws \
    && colcon build --packages-select mavros_msgs

# We then need to build the bridge from source
#  (So we probably don't need to use the ros1-bridge image...)
# Remove the preinstalled version
RUN apt-get remove -y ros-foxy-ros1-bridge
# Get the source
RUN git clone -b foxy https://github.com/ros2/ros1_bridge /bridge_ws/src/ros1_bridge
# Build with ROS2 mavros_msgs on path
RUN cd /bridge_ws \
    && . /opt/ros/${ROS1_DISTRO}/setup.sh \
    && . /opt/ros/${ROS2_DISTRO}/setup.sh \
    && . /ros_ws/install/local_setup.sh \
    && export CMAKE_PREFIX_PATH=$AMENT_PREFIX_PATH:$CMAKE_PREFIX_PATH \
    && colcon build --packages-select ros1_bridge --cmake-force-configure

COPY mavros_bridge.launch.xml /ros_ws/launch/
COPY run_ros1.sh /ros_ws/scripts/

COPY ros_entrypoint.sh /ros_entrypoint.sh

ENV MAVROS_FCU_URL="udp://:14540@"
ENV MAVROS_TGT_SYSTEM=1
ENV MAVROS_TGT_FIRMWARE="px4"

WORKDIR /ros_ws

COPY mavros_setup.sh .

CMD [ "ros2", "launch", "launch/mavros_bridge.launch.xml" ]
