# Setup common base for simulator images
FROM ros:foxy-ros-base-focal

###
# Steps from gazebo:gzserver11 Dockerfile
###

RUN apt-get update \
    && apt-get install --no-install-recommends -y \
      lsb-release \
    && rm -rf /var/lib/apt/lists/*

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys D2486D2DD83DB69272AFE98867170598AF249743

# setup sources.list
RUN . /etc/os-release \
    && echo "deb http://packages.osrfoundation.org/gazebo/$ID-stable `lsb_release -sc` main" > /etc/apt/sources.list.d/gazebo-latest.list

# install gazebo packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    gazebo11=11.3.0-1* \
    && rm -rf /var/lib/apt/lists/*

# setup environment
EXPOSE 11345

###
# End of gazebo:gzserver11 steps
###

# Install gazebo_ros
RUN apt-get update \
    && apt-get install --no-install-recommends -y \
      ros-foxy-gazebo-ros \
    && rm -rf /var/lib/apt/lists/*

# Install gzweb dependencies
RUN apt-get update \
    && apt-get install --no-install-recommends -y \
      build-essential \
      cmake \
      curl \
      git \
      imagemagick \
      libboost-dev \
      libjansson-dev \
      libtinyxml-dev \
      xvfb \
    && rm -rf /var/lib/apt/lists/*

# Install nvm
RUN curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.37.2/install.sh | bash

# Install node
# Use bash shell for this next bit...
SHELL [ "/usr/bin/bash", "-c" ]
RUN source /root/.nvm/nvm.sh && nvm install 14

# Fetch gzweb
ENV GZWEB_WS /root/gzweb
# Patched version
#RUN git clone -b gzweb_1.4.0-gazebo11 https://github.com/eurogroep/gzweb.git ${GZWEB_WS}
RUN git clone -b gzweb_1.4.1 https://github.com/osrf/gzweb ${GZWEB_WS}

# Setup ports
EXPOSE 8080
EXPOSE 7681

# Add launch files
COPY gzweb.launch.xml /ros_ws/
COPY gzweb_entrypoint.sh /

# Need to wait until models are installed before running the deploy step
CMD [ "gzweb_entrypoint.sh" ]