# Build & Run

With `make` installed, run `make` in the root folder to build the docker images
Use `make run` to start the gazebo, sitl and mavros containers

Go to http://127.0.0.1:8080 to (hopefully) see the simulator

# Details

The root `Makefile` delegates to submakes to build various images:
 - `starling-sim-base-core` containing Gazebo, ROS2 and gzweb
 - `starling-sim-base-px4` built on top of `-core`, containing the PX4 Gazebo
    plugins
 - `starling-sim-clover2` built on top of `-px4`, containing the Clover model
    and textures
 - `starling-sim-px4-sitl` containing just the PX4 SITL binary

A `docker-compose` file sets up three containers; one for the Gazebo server,
one for the PX4 SITL, and one for mavros.

The Gazebo server runs in the `starling-sim-clover2` image with port 8080
published to allow web connection. The default command for this image is a
`ros2 launch` for `clover2.launch.xml`. This includes the lower layer launch
script which starts Gazebo and gzweb. Additionally, it runs a `spawn_entity`
script to spawn an instance of the Clover model in Gazebo. This model comes
from the `/robot_description` topic which is published by ROS2's
`robot_state_publisher`.

## Scaling

Scaling this setup involves spawning additional models on the Gazebo server and
running additional SITL and MAVROS instances. The `px4-sitl` image provides an
environment variable: `PX4_INSTANCE`. By default this has a value of zero.
Incrementing this will also increment the port numbers for the simulator host
and the mavlink outputs. This means the model needs to be spawned with this
in mind. The `PX4_SITL_PORT` variable for the `sim-clover2` image provides the
hook to adjust the model spawning. The `fcu_url` argument to the launch file
for the `mavros` image provides the adjustment for the MAVROS side.

Example of how scaling might work is in `docker-compose.multiple.yml` but this
hasn't been tested.

# Ideas

Ideally, baking the model into the image would not be required. Unfortunately,
gzweb needs model-specific assets before it can display a model. Custom plugins
may also be needed by the Gazebo server before a model can be loaded. Volumes
might provide a neater, if less user friendly way to do this.

## `ros.env.d`
Adding a folder to a `/ros.env.d` could provide an extendable way to add to the
ros environment. e.g. including additional model/plugin paths for Gazebo.