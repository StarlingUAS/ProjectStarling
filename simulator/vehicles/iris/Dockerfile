ARG VERSION=latest
ARG REGISTRY
FROM ${REGISTRY}uobflightlabstarling/starling-sim-base-px4:${VERSION}

# Copy in the vehicle launch file
COPY iris.launch.xml /ros_ws/launch/
COPY spawn_iris.sh /ros_ws/
COPY vehicle_setup.sh /ros.env.d/10_vehicle/setup.bash

ENV PX4_SITL_PORT 4560
ENV PX4_INSTANCE 0
ENV PX4_SIM_MODEL iris

RUN echo 'export PX4_SITL_PORT=$((PX4_SITL_PORT + PX4_INSTANCE))' >> /ros.env

WORKDIR /ros_ws

# Launch gazebo and spawn model
CMD [ "ros2", "launch", "launch/iris.launch.xml" ]
