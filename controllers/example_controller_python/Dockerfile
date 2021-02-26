FROM uobflightlabstarling/starling-controller-base

COPY example_controller_python /ros_ws/src/

RUN . /ros_ws/install/setup.sh \
    && colcon build

# This may be needed for python print output in kubernetes logging
# Either set to 1 or zero
# ENV PYTHONBUFERED=1 

CMD [ "ros2", "run", "example_controller_python", "controller" ]
