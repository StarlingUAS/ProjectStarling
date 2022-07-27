ARG VERSION=latest
ARG REGISTRY
FROM ${REGISTRY}uobflightlabstarling/starling-controller-base:${VERSION}

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        libboost-system-dev \
        git \
    && rm -rf /var/lib/apt/lists/*

# Copies all the nodes into the container
COPY ping_monitor /ros_ws/src
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && export CMAKE_PREFIX_PATH=$AMENT_PREFIX_PATH:$CMAKE_PREFIX_PATH \
    && cd /ros_ws \
    && colcon build\
    && rm -r build

COPY ping_monitor/run.sh /ros_ws
CMD ["./run.sh"]