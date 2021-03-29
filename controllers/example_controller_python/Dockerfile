ARG VERSION=latest
FROM uobflightlabstarling/starling-controller-base:${VERSION}

COPY example_controller_python /ros_ws/src/

RUN . /ros_ws/install/setup.sh \
    && colcon build

# This may be needed for python print output in kubernetes logging
# Either set to 1 or zero
ENV PYTHONBUFERED=1 

CMD [ "ros2", "launch", "example_controller_python", "example_fly_all.launch.py" ]
