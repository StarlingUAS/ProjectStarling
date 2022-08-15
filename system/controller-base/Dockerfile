# FROM ros:foxy-ros-base-focal (18/03/2022: https://github.com/docker-library/repo-info/blob/c659a4c50ed712fcc52cfe86728ae543d42f5dfb/repos/ros/remote/foxy-ros-base-focal.md)
FROM ros@sha256:953899ce6b949c54ee4f76b15544e7fe139ca18319918f52044500237a864111

# Overwrite existing entrypoint script to allow additional sources from /ros.env and /ros.env.d
RUN touch /ros.env
RUN mkdir /ros.env.d
COPY ros_entrypoint.sh /

# Install the ROS2 version of mavros_msgs
# Clone the repo
RUN apt-get update \
    && apt-get install -y ros-${ROS_DISTRO}-mavros-msgs \
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

ENV USE_SIMULATED_TIME false

WORKDIR /ros_ws
