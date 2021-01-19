FROM starling-sim-base-px4

# Fetch CoEx Clover model files
RUN git clone -b v0.21.1 https://github.com/CopterExpress/clover /src/clover

# Launch gazebo and spawn model
CMD ["ros2", "launch"]
