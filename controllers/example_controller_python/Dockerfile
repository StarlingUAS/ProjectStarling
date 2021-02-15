FROM starling-controller-base

COPY example_controller_python /ros_ws/src/

RUN . /ros_ws/install/setup.sh \
    && colcon build

CMD [ "ros2", "run", "example_controller_python", "controller" ]
