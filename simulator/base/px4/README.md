The simulator is setup in two parts, the actual SITL binary and the Gazebo
plugins. The Gazebo plugins need to be volume mounted in the base image
and have a envvar set to point to them. The SITL itself can exist in a
separate container.

The SITL container expects `PX4_OFFBOARD_HOST` or `PX4_OFFBOARD_IP` to be set
to control which IP offboard packets are sent to.

The SITL listens on 18570 + $PX4_INSTANCE for GCS traffic. This port should
be exposed to host for using QGC