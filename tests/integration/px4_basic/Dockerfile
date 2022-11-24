FROM ros:foxy-ros-base-focal

RUN apt-get update \
	&& apt-get install --no-install-recommends -y \
		ros-foxy-mavros-msgs \
	&& apt-get clean

WORKDIR /ros_ws
COPY test_script.sh ./

CMD [ "./test_script.sh" ]
