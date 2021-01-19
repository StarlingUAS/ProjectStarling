FROM starling-sim-base-px4

# Fetch CoEx Clover model files
RUN git clone -b v0.21.1 https://github.com/CopterExpress/clover /src/clover

RUN apt-get update \
    && apt-get install --no-install-recommends -y \
      ros-foxy-xacro \
    && rm -rf /var/lib/apt/lists/

ENV ROS_PACKAGE_PATH /opt/ros/foxy/share:/src/clover

COPY ./setup_* /scripts/

# Launch gazebo and spawn model
CMD ["/scripts/setup_sim.sh"]
