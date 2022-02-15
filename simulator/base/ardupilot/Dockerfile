# Add the ArduPilot plugin to the Gazebo image
# Produces an image based on the core gazebo image that includes the ArduPilot-specific plugins

ARG VERSION=latest
ARG REGISTRY

FROM ${REGISTRY}uobflightlabstarling/starling-sim-base-core:${VERSION}

# Clone and build the AP Gazebo plugins
RUN git clone https://github.com/khancyr/ardupilot_gazebo \
    && cd ardupilot_gazebo \
    && mkdir build \
    && cd build \
    && cmake .. \
    && make -j4 \
    && make install

# Setup a ros.env.d folder for AP to add to the resource paths for Gazebo
RUN mkdir -p /ros.env.d/00_ardupilot \
    && echo 'export GAZEBO_MODEL_PATH=/ardupilot_gazebo/models:${GAZEBO_MODEL_PATH}' >> /ros.env.d/00_ardupilot/setup.bash \
    && echo 'export GAZEBO_RESOURCE_PATH=/ardupilot_gazebo/worlds:${GAZEBO_RESOURCE_PATH}' >> /ros.env.d/00_ardupilot/setup.bash

COPY sim_setup.sh /ros.env.d/00_setup/setup.bash

ENV AP_SYSID 1
ENV AP_SITL_ADDRESS 127.0.0.1
# AP_SITL_HOST
