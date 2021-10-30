ARG VERSION=latest
ARG REGISTRY
FROM ${REGISTRY}uobflightlabstarling/starling-sim-ardupilot-gazebo:${VERSION}

# Copy in the vehicle launch file
COPY iris.launch.xml /ros_ws/launch/
COPY spawn_iris.sh /ros_ws/
<<<<<<< HEAD
COPY models /ardupilot_gazebo/models/

COPY gimbal_plugin /ros_ws/src/gimbal_plugin

# Build with ROS2 on path
RUN cd /ros_ws \
    && . /opt/ros/foxy/setup.sh \
    && export CMAKE_PREFIX_PATH=$AMENT_PREFIX_PATH:$CMAKE_PREFIX_PATH \
    && colcon build --cmake-force-configure \
    && rm -r build

# Add plugins to path manually
RUN echo 'export GAZEBO_PLUGIN_PATH=/ros_ws/install/gimbal_plugin/lib:${GAZEBO_PLUGIN_PATH}' >> /ros.env.d/ardupilot/setup.bash
=======

ENV AP_SITL_ADDRESS 127.0.0.1
# AP_SITL_HOST

# Copy in the xacro & bash file to setup the model with a specific address
RUN mkdir /ros.env.d/iris_ap
COPY model.sdf.xacro /ros.env.d/iris_ap/model.sdf.xacro
COPY iris_setup.sh /ros.env.d/iris_ap/setup.bash
>>>>>>> dev

WORKDIR /ros_ws

# Launch gazebo and spawn model
CMD [ "ros2", "launch", "launch/iris.launch.xml" ]
