# Download the PX4 repo with a specific version tag
# Here to DRY out the main Dockerfile and sitl.Dockerfile

# Perform multistage build to avoid the pile of PX4 deps
FROM px4io/px4-dev-simulation-focal as px4builder

# Fetch the PX4 code
RUN git clone --recurse-submodules -j4 --depth 1 --shallow-submodules -j4 \
    -b v1.11.3 https://github.com/PX4/PX4-Autopilot /src/PX4-Autopilot

# Switch to the workdir
WORKDIR /src/PX4-Autopilot
