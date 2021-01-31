FROM ros:foxy-ros1-bridge

RUN apt-get update \
    && apt-get install --no-install-recommends -y \
      ros-${ROS1_DISTRO}-mavros \
      ros-${ROS1_DISTRO}-mavros-extras \
    && rm -rf /var/lib/apt/lists/

# Part of MAVROS setup
RUN /opt/ros/${ROS1_DISTRO}/lib/mavros/install_geographiclib_datasets.sh

COPY mavros_bridge.launch.xml /ros_ws/launch/
COPY run_ros1.sh /ros_ws/scripts/

WORKDIR /ros_ws

CMD [ "ros2", "launch", "launch/mavros_bridge.launch.xml", "fcu_url:=udp://@" ]
