FROM ros:noetic

RUN apt-get update \
    && apt-get install --no-install-recommends -y \
      ros-${ROS_DISTRO}-mavros \
      ros-${ROS_DISTRO}-mavros-extras \
    && rm -rf /var/lib/apt/lists/

# Part of MAVROS setup
RUN /opt/ros/${ROS_DISTRO}/lib/mavros/install_geographiclib_datasets.sh

CMD [ "roslaunch", "mavros", "px4.launch" ]
