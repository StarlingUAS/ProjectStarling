# Perform multistage build to avoid a pile of ardupilot deps
FROM ardupilot/ardupilot-dev-chibios as ardupilot_builder

# Fetch ArduPilot
RUN git clone --recurse-submodules --depth 1 --shallow-submodules -j4 \
    -b Copter-4.0.5 https://github.com/ArduPilot/ardupilot /src/ardupilot

# Build the SITL
WORKDIR /src/ardupilot
RUN ./waf configure && ./waf copter

# Fetch the ArduPilot gazebo plugin
RUN git clone --depth 1 https://github.com/khancyr/ardupilot_gazebo /src/ardupilot_gazebo

FROM starling-sim-base-core

# Copy ardupilot sources from builder stage
COPY --from=ardupilot_builder /src /src/

# Build & install the gazebo link
RUN cd /src/ardupilot_gazebo \
    && mkdir build \
    && cd build \
    && cmake .. \
    && make \
    && make install

CMD [ "gzserver", "worlds/iris_arducopter_runway.world"]

