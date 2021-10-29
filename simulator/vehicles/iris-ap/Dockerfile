ARG VERSION=latest
ARG REGISTRY
FROM ${REGISTRY}uobflightlabstarling/starling-sim-ardupilot-gazebo:${VERSION}

# Copy in the vehicle launch file
COPY iris.launch.xml /ros_ws/launch/
COPY spawn_iris.sh /ros_ws/

ENV AP_SITL_ADDRESS 127.0.0.1

# Copy in the xacro file to setup the model with a specific address
COPY model.sdf.xacro /ardupilot_gazebo/models/iris_with_ardupilot/
RUN echo 'xacro /ardupilot_gazebo/models/iris_with_ardupilot/model.sdf.xacro fdm_addr:=${AP_SITL_ADDRESS} > /ardupilot_gazebo/models/iris_with_ardupilot/model.sdf' \
    >> /ros.env.d/ardupilot/setup.bash


WORKDIR /ros_ws

# Launch gazebo and spawn model
CMD [ "ros2", "launch", "launch/iris.launch.xml" ]
