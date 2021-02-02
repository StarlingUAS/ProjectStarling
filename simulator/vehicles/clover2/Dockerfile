FROM starling-sim-base-px4

# Copy in the new clover_description files
COPY clover_description /ros_ws/src/clover_description/

SHELL [ "/ros_entrypoint.sh", "bash", "-c" ]

# Build the package so ROS2 can find it
RUN cd /ros_ws && . /ros.env && colcon build

# Add ros_ws setup to ros.env for entrypoint
RUN echo "source /ros_ws/install/setup.bash" >> /ros.env

# Build SDF from URDF for gzweb
# Should get picked up in gzweb deploy
RUN mkdir -p /ros_ws/models && /ros_ws/tools/createModel.sh \
    /ros_ws/models/clover_description \
    /ros_ws/src/clover_description/media \
    /ros_ws/src/clover_description/meshes \
    -- /ros_ws/src/clover_description/urdf/clover/clover4.xacro \
        main_camera:=false rangefinder:=false led:=false gps:=false maintain_camera_rate:=false use_clover_physics:=false
ENV GAZEBO_MODEL_PATH /ros_ws/models:${GAZEBO_MODEL_PATH}

# Copy in the vehicle launch file
COPY clover2.launch.xml /ros_ws/launch/

ENV PX4_SITL_PORT 4560
ENV PX4_INSTANCE 0

RUN echo 'export PX4_SITL_PORT=$((PX4_SITL_PORT + PX4_INSTANCE))' >> /ros.env

WORKDIR /ros_ws

# Launch gazebo and spawn model
CMD [ "ros2", "launch", "launch/clover2.launch.xml" ]
