FROM ros:foxy-ros-base-focal

# Install nvm
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        curl \
        ros-foxy-rosbridge-suite \
        ros-foxy-mavros-msgs \
        ros-foxy-gazebo-msgs \
    && rm -rf /var/lib/apt/lists/*



ENV ROSBRIDGE_PORT 55555

# Expose websocket server
EXPOSE ${ROSBRIDGE_PORT}

# Set initial command
CMD [ "sh", "-c", "ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=${ROSBRIDGE_PORT}" ]