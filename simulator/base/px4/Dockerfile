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

# Copy launch script across
COPY sim.launch.xml /ros_ws/launch/

# Add ROS and PX4/Gazebo environment setup to a ros.env that is sourced by entrypoint
RUN touch /ros.env && echo "source /opt/ros/foxy/setup.bash" >> /ros.env
RUN echo "source /src/PX4-Autopilot/Tools/setup_gazebo.bash /src/PX4-Autopilot /src/PX4-Autopilot/build/px4_sitl_default" >> /ros.env

# Copy entrypoint script
COPY ros_entrypoint.bash /

WORKDIR /ros_ws

ENTRYPOINT [ "/ros_entrypoint.bash" ]

CMD [ "ros2", "launch", "/ros_ws/launch/sim.launch.xml" ]
