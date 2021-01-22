FROM gazebo:libgazebo11

RUN apt-get update \
    && apt-get install --no-install-recommends -y \
      build-essential \
      git \
    && rm -rf /var/lib/apt/lists/*

# Install the ArduPilot gazebo plugin
RUN git clone https://github.com/khancyr/ardupilot_gazebo \
    && cd ardupilot_gazebo \
    && mkdir build \
    && cd build \
    && cmake .. \
    && make \
    && make install

CMD [ "gzserver", "worlds/iris_arducopter_runway.world"]

