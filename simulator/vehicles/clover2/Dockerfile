FROM starling-sim-base-px4

RUN apt-get update \
    && apt-get install --no-install-recommends -y \
      ros-foxy-xacro \
    && rm -rf /var/lib/apt/lists/

# Copy in the new clover_description files
COPY clover_description /ros_ws/src/clover_description/

SHELL [ "/ros_entrypoint.sh", "bash", "-c" ]

# Build the package so ROS2 can find it
RUN cd /ros_ws && . /ros.env && colcon build

# Copy in the vehicle launch file
COPY clover2.launch.xml /ros_ws/launch/

# Add ros_ws setup to ros.env for entrypoint
RUN echo "source /ros_ws/install/setup.bash" >> /ros.env

# Launch gazebo and spawn model
CMD [ "ros2", "launch", "/ros_ws/launch/clover2.launch.xml" ]
