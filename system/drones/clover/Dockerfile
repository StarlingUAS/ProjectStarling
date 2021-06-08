ARG VERSION=latest
ARG REGISTRY
FROM ${REGISTRY}uobflightlabstarling/starling-controller-base:${VERSION}

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        libboost-system-dev \
        git \
        python3-pip \
    && rm -rf /var/lib/apt/lists/*

## Install Raspberry Pi GPIO drivers
RUN pip3 install RPi.GPIO

## Geographic Lib if neccessary
# RUN git clone \
#         --filter=blob:none --no-checkout\
#         --single-branch --branch ros2 \
#         https://github.com/ros-geographic-info/geographic_info.git \
#         /ros_ws/src/geographic_info \
#     && cd /ros_ws/src/geographic_info \
#     && git checkout master --geographic_msgs

## mavros_msgs should already be installed
## Install ROS2 version just in case
# RUN git clone \
#         --no-checkout --depth 1\
#         --single-branch --branch ros2 \
#         https://github.com/mavlink/mavros.git \
#         /ros_ws/src/mavros \
#     && cd /ros_ws/src/mavros \
#     && git sparse-checkout init --cone \
#     && git sparse-checkout set mavros_msgs

## My own files for clover core libraries
RUN git clone https://github.com/UoBFlightLab/clover_ros2_pkgs.git \
        /ros_ws/src/clover_ros2_pkgs

WORKDIR /ros_ws

RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && export CMAKE_PREFIX_PATH=$AMENT_PREFIX_PATH:$CMAKE_PREFIX_PATH \
    && cd /ros_ws \
    && colcon build --packages-select \
        # mavros_msgs \
        led_msgs \
        ws281x \
        clover_ros2 \
    && rm -r build


CMD [ "ros2", "launch", "clover_ros2", "clover.launch.xml"]