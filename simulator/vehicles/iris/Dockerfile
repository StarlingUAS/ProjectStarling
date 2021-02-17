FROM starling-sim-base-px4

# Copy in the vehicle launch file
COPY iris.launch.xml /ros_ws/launch/

ENV PX4_SITL_PORT 4560
ENV PX4_INSTANCE 0

RUN echo 'export PX4_SITL_PORT=$((PX4_SITL_PORT + PX4_INSTANCE))' >> /ros.env

WORKDIR /ros_ws

# Launch gazebo and spawn model
CMD [ "ros2", "launch", "launch/iris.launch.xml" ]
