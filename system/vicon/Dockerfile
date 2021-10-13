FROM ros:foxy-ros-base

# Copy in the source
COPY vicon_udp /ros_ws/src/vicon_udp

WORKDIR /ros_ws

# Install dependencies
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && apt-get update \
    && rosdep install --from-paths src \
    && rm -rf /var/lib/apt/lists/

# Copy in additional launch file
COPY vicon_single.launch.xml /ros_ws/src/vicon_udp/launch/

# Build the package
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && colcon build

# Setup the new entrypoint
COPY entrypoint.sh /ros_entrypoint.sh

# Default Vicon UDP port is 51001
EXPOSE 51001/udp

CMD [ "ros2", "launch", "vicon_udp", "vicon_single.launch.xml" ]
