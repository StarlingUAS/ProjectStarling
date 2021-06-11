ARG VERSION=latest
ARG REGISTRY

# Perform multistage build to avoid a pile of ardupilot deps
FROM ardupilot/ardupilot-dev-chibios as ardupilot_builder

ARG VEHICLE=copter
ARG BRANCH=ArduCopter-stable

# Fetch ArduPilot
RUN git clone --recurse-submodules --depth 1 --shallow-submodules -j4 \
    -b ${BRANCH} https://github.com/ArduPilot/ardupilot /src/ardupilot

# Build the SITL
WORKDIR /src/ardupilot
RUN ./waf configure && ./waf ${VEHICLE}

WORKDIR /home/root
COPY entrypoint.sh /home/root/

# The VEHICLE environment variable should not be overridden as
#  the build will not exist for it
ENV VEHICLE=${VEHICLE}
ENV AP_SYSID_BASE 0
ENV AP_SYSID=0

ENTRYPOINT [ "/home/root/entrypoint.sh" ]
