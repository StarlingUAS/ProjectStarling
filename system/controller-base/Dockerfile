FROM ros:foxy-ros-base-focal

# Overwrite existing entrypoint script to allow additional sources from /ros.env and /ros.env.d
RUN touch /ros.env
RUN mkdir /ros.env.d
COPY ros_entrypoint.sh /

# Install the ROS2 version of mavros_msgs
# Clone the repo
RUN apt-get update \
    && apt-get install -y ros-${ROS_DISTRO}-mavros-msgs ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# # Build the package
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && mkdir /ros_ws \
    && cd /ros_ws \
    && colcon build \
    && rm -r build

COPY base_env_setup.sh /ros.env.d/controller_base/setup.bash

# Add custom ROS DDS configuration (force UDP always)
COPY fastrtps_profiles.xml /ros_ws/
ENV FASTRTPS_DEFAULT_PROFILES_FILE /ros_ws/fastrtps_profiles.xml

# Add custom cylonedds configuration and set as default
COPY cyclonedds.xml /ros_ws/
ENV CYCLONEDDS_URI /ros_ws/cyclonedds.xml
ENV RMW_IMPLEMENTATION rmw_cyclonedds_cpp

ENV USE_SIMULATED_TIME false

WORKDIR /ros_ws
