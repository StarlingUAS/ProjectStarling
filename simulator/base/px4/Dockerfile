# Get the PX4 dev environment
FROM px4io/px4-dev-ros2-foxy:2020-11-18

RUN apt-get update \
    && apt-get install --no-install-recommends -y \
      ros-foxy-gazebo-ros \
    && rm -rf /var/lib/apt/lists/*

RUN git clone --recurse-submodules -j4 -b v1.11.3 https://github.com/PX4/PX4-Autopilot /src/PX4-Autopilot

WORKDIR /src/PX4-Autopilot

# Build the SITL and the gazebo link
RUN make px4_sitl
RUN make px4_sitl sitl_gazebo

# Add entrypoint to setup PX4 gazebo plugins
COPY sim_entrypoint.sh /
ENTRYPOINT ["/sim_entrypoint.sh"]

CMD [ "ros2", "launch", "gazebo_ros", "gzserver.launch.py" "world_name:=worlds/empty.world" ]
