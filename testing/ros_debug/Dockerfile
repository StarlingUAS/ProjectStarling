FROM ros:foxy

# Add new user ctrl_user to the system
RUN useradd --create-home ctrl_user
USER ctrl_user

# Change working directory to ctrl_user directory
WORKDIR /home/ctrl_user

# Copy workspace into user root
COPY --chown=ctrl_user workspace .

# RUN ./setup_script.sh


