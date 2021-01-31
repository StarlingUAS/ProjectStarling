The simulator is setup in two parts, the actual SITL binary and the Gazebo
plugins. The Gazebo plugins need to be volume mounted in the base image
and have a envvar set to point to them. The SITL itself can exist in a
separate container.