# Build & Run

Use `./build_images.sh` to build the docker images
Use `./run_docker.sh` to start the clover2 simulator

Go to http://127.0.0.1:8080 to (hopefully) see the simulator

# Details

The build script builds a set of images:

 - `starling-sim-base-core` containing Gazebo, ROS2 and gzweb
 - `starling-sim-base-px4` built on top of `-core`, containing the PX4 SITL and
    Gazebo plugins
 - `starling-sim-clover2` built on top of `-px4`, containing the Clover model
    and textures

The run script runs the `starling-sim-clover2` image with port 8080 published
to allow web connection. The default command for this image is a `ros2 launch`.
This calls the lower layer launch scripts which start Gazebo and gzweb, and
start the PX4 SITL. It also runs a `spawn_entity` script to spawn an instance
of the Clover model in gazebo.

# Ideas

Ideally, baking the model into the image would not be required. Unfortunately,
gzweb needs model-specific assets before it can display a model. Custom plugins
may also be needed by the Gazebo server before a model can be loaded. Volumes
might provide a neater, if less user friendly way to do this.

## `ros.env.d`
Adding a folder to a `/ros.env.d` could provide an extendable way to add to the
ros environment. e.g. including additional model/plugin paths for Gazebo.