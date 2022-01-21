FROM ros:foxy-ros-base-focal

# Install rosbridge suite and associated messages
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        curl \
        ros-foxy-rosbridge-suite \
        ros-foxy-mavros-msgs \
        ros-foxy-gazebo-msgs \
    && rm -rf /var/lib/apt/lists/*


# Add custom ROS DDS configuration (force UDP always)
COPY fastrtps_profiles.xml /ros_ws/
ENV FASTRTPS_DEFAULT_PROFILES_FILE /ros_ws/fastrtps_profiles.xml

COPY rosbridge_websocket_launch.xml /ros_ws/
COPY ros_entrypoint.sh /
COPY run.sh /

ENV ROSBRIDGE_PORT 9090
ENV MSGS_WS "/msgs_ws"

# Expose websocket server
EXPOSE ${ROSBRIDGE_PORT}

# Set initial command
CMD ["./run.sh"]