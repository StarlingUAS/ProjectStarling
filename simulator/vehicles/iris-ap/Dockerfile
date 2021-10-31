ARG VERSION=latest
ARG REGISTRY
FROM ${REGISTRY}uobflightlabstarling/starling-sim-ardupilot-gazebo:${VERSION}

# Copy in the vehicle launch file
COPY iris.launch.xml /ros_ws/launch/
COPY spawn_iris.sh /ros_ws/

# Build gimbal plugin with ROS2 on path and add plugin to path
COPY gimbal_plugin /ros_ws/src/gimbal_plugin
RUN cd /ros_ws \
    && . /opt/ros/foxy/setup.sh \
    && export CMAKE_PREFIX_PATH=$AMENT_PREFIX_PATH:$CMAKE_PREFIX_PATH \
    && colcon build --cmake-force-configure \
    && rm -r build

# Copy in the xacro & bash file to setup the model
# Using ros.env.d automatic sourcing on entrypoint (see /simulator/base/core/Dockerfile)
COPY models/gimbal_small_2d /ros.env.d/02_gimbal_small_2d
COPY models/iris_ap /ros.env.d/03_iris_ap


RUN echo 'export GAZEBO_PLUGIN_PATH=/ros_ws/install/gimbal_plugin/lib:${GAZEBO_PLUGIN_PATH}' >> /ros.env.d/02_gimbal_small_2d/setup.bash


WORKDIR /ros_ws

# Launch gazebo and spawn model
CMD [ "ros2", "launch", "launch/iris.launch.xml" ]
