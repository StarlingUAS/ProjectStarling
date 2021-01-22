# Perform multistage build to avoid the pile of PX4 deps
FROM px4io/px4-dev-simulation-focal as px4builder

# Fetch the PX4 code
RUN git clone --recurse-submodules -j4 -b v1.11.3 https://github.com/PX4/PX4-Autopilot /src/PX4-Autopilot

# Build the SITL and the gazebo link
WORKDIR /src/PX4-Autopilot
RUN make px4_sitl
RUN make px4_sitl sitl_gazebo

FROM starling-sim-base-core

# Copy built PX4 repo into this image
COPY --from=px4builder /src /src

# Copy launch script across
COPY sim.launch.xml /ros_ws/launch/

# Add PX4/Gazebo environment setup to a ros.env that is sourced by entrypoint
RUN touch /ros.env && echo "source /src/PX4-Autopilot/Tools/setup_gazebo.bash /src/PX4-Autopilot /src/PX4-Autopilot/build/px4_sitl_default" >> /ros.env

# Overwrite existing entrypoint script
COPY ros_entrypoint.sh /

WORKDIR /ros_ws

ENTRYPOINT [ "/ros_entrypoint.sh" ]

CMD [ "ros2", "launch", "/ros_ws/launch/sim.launch.xml" ]
