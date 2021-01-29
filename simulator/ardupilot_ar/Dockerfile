FROM arthurrichards77/ardupilot-sitl-docker:latest

# The default params don't stream anything to the later serial ports
# This adds the additional parameters to enable that by default

COPY --chown=pilot additional_param.parm /home/pilot/

RUN cat /home/pilot/ardupilot/Tools/autotest/default_params/copter.parm >> /home/pilot/additional_param.parm

ENV SITL_OPTS='--model=quad --defaults=/home/pilot/additional_param.parm'