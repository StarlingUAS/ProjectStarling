ARG VERSION=latest
ARG REGISTRY
FROM ${REGISTRY}uobflightlabstarling/starling-sim-ardupilot-gazebo:${VERSION}

# Copy in the vehicle launch file
COPY iris.launch.xml /ros_ws/launch/
COPY spawn_iris.sh /ros_ws/
COPY models /ardupilot_gazebo/models/

# RUN echo 'export PX4_SITL_PORT=$((PX4_SITL_PORT + PX4_INSTANCE))' >> /ros.env

WORKDIR /ros_ws

# Launch gazebo and spawn model
CMD [ "ros2", "launch", "launch/iris.launch.xml" ]
