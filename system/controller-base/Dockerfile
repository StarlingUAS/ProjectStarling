FROM ros:foxy-ros-base-focal

# Install the ROS2 version of mavros_msgs
# Clone the repo
RUN git clone -b mavros2 https://github.com/rob-clarke/mavros /ros_ws/src/mavros

# Fetch dependencies
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && apt-get update \
    && rosdep install --from-paths /ros_ws/src/mavros/mavros_msgs \
    && rm -rf /var/lib/apt/lists/

# Build the package
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && export CMAKE_PREFIX_PATH=$AMENT_PREFIX_PATH:$CMAKE_PREFIX_PATH \
    && cd /ros_ws \
    && colcon build --packages-select mavros_msgs \
    && rm -r build

COPY ros_entrypoint.sh /ros_entrypoint.sh

# Add custom ROS DDS configuration (force UDP always) 
COPY fastrtps_profiles.xml /ros_ws/
ENV FASTRTPS_DEFAULT_PROFILES_FILE /ros_ws/fastrtps_profiles.xml

WORKDIR /ros_ws
