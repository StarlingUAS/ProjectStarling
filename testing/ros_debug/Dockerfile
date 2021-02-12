FROM ros:foxy

# Add new user ctrl_user to the system
RUN useradd --create-home ctrl_user
USER ctrl_user

# Change working directory to ctrl_user directory
WORKDIR /home/ctrl_user

# Copy workspace into user root
COPY --chown=ctrl_user workspace workspace
COPY --chown=ctrl_user ros_entrypoint.sh .
# COPY --chown=ctrl_user setup_script.sh .
# RUN ./setup_script.sh

WORKDIR /home/ctrl_user/workspace/dev_ws
RUN /bin/bash -c '\
    source /opt/ros/foxy/setup.bash && \ 
    colcon build'

WORKDIR /home/ctrl_user
ENTRYPOINT [ "/home/ctrl_user/ros_entrypoint.sh" ]
CMD ["ros2", "run", "ros_debug_nodes", "talker"]


