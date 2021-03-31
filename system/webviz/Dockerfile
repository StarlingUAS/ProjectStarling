FROM ros:foxy-ros-base-focal

# Install nvm
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        curl \
    && rm -rf /var/lib/apt/lists/*

RUN curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.37.2/install.sh | bash


# Install node
# Use bash shell for this next bit...
SHELL [ "/usr/bin/bash", "-c" ]
RUN source /root/.nvm/nvm.sh && nvm install 12

# Get ros2-web-bridge
RUN git clone --recurse-submodules -j4 --depth 1 --shallow-submodules https://github.com/RobotWebTools/ros2-web-bridge /ros_ws/src/ros2-web-bridge

# Change the working directory
WORKDIR /ros_ws/src/ros2-web-bridge

# Install ros2-web-bridge
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && source /root/.nvm/nvm.sh && npm install

# Get Webviz
RUN git clone --recurse-submodules -j4 --depth 1 --shallow-submodules https://github.com/cruise-automation/webviz.git /ros_ws/src/webviz

# Change the working directory 
WORKDIR /ros_ws/src/webviz

# Install webviz
RUN source /root/.nvm/nvm.sh && npm run bootstrap && npm run build-static-webviz

# Copy over the launch scripts
COPY node_entrypoint.sh /node_entrypoint.sh

# Change the working directory
WORKDIR /ros_ws

# Expose html server
EXPOSE 3000
# Expose websocket server
EXPOSE 9090

# Set initial command
CMD [ "npm", "run", "serve-static-webviz" ]