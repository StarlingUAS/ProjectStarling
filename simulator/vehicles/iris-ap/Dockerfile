ARG VERSION=latest
ARG REGISTRY
FROM ${REGISTRY}uobflightlabstarling/starling-sim-ardupilot-gazebo:${VERSION}

# Copy in the vehicle launch file
COPY iris.launch.xml /ros_ws/launch/
COPY spawn_iris.sh /ros_ws/

ENV AP_SITL_ADDRESS 127.0.0.1
# AP_SITL_HOST

# Copy in the xacro & bash file to setup the model with a specific address
RUN mkdir /ros.env.d/iris_ap
COPY model.sdf.xacro /ros.env.d/iris_ap/model.sdf.xacro
COPY iris_setup.sh /ros.env.d/iris_ap/setup.bash

WORKDIR /ros_ws

# Launch gazebo and spawn model
CMD [ "ros2", "launch", "launch/iris.launch.xml" ]
