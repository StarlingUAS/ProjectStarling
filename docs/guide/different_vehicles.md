# Different Vehicles in Simulation

With the way the system is setup, it can be quite simple to run another vehicle in simulation, this page shows two different methods of achieving this effect.

[TOC]

## Using Existing Models

To reduce the size of the containers, they only contain the minimal number of vehicle models

- For PX4 SITL container, there are only two models (1) iris (2) r1_rover. 
- For Ardupilot SITL container, not many exist ... see the next section for how to add your own.

For those which already exist it is simply a case of setting the `PX4_SIM_MODEL` envrionment variable when you run

- `uobflightlabstarling/starling-sim-iris` in order to specify the model which will be rendered. By default this is `iris`
- `uobflightlabstarling/starling-sim-px4-sitl:latest` in order to ensure that the correct SITL parameters have been loaded. By default this is also `iris`

### Example: Rover


Example docker-compose to run the [`r1-rover`](https://docs.px4.io/master/en/simulation/gazebo_vehicles.html#differential-ugv)
```yaml
version: '3'
services:
    simhost: 
        image: uobflightlabstarling/starling-sim-iris
        environment:
        - "PX4_SIM_MODEL=r1-rover" #new!
        ports:
        - "8080:8080"
    sitl:
        image: uobflightlabstarling/starling-sim-px4-sitl:latest
        environment:
        - "PX4_SIM_HOST=simhost"
        - "PX4_OFFBOARD_HOST=mavros"
        - "PX4_SIM_MODEL=r1-rover" # new! 
        ports:
        - "18570:18570/udp"
    mavros:
        image: uobflightlabstarling/starling-mavros:latest
        command: ros2 launch launch/mavros_bridge.launch.xml
        environment:
        - "MAVROS_TGT_SYSTEM=1"
        - "MAVROS_FCU_IP=0.0.0.0"
        - "MAVROS_GCS_URL=tcp-l://0.0.0.0:5760"
        ports:
        - "5760:5760"
```

### Using existing models which are not included in the container

As mentioned above, there are a number of models which should be included, but have been deleted by default to reduce the size of the container. 

For example for the PX4 SITL container, the following models should all exist: [github link](https://github.com/PX4/PX4-SITL_gazebo/tree/822050a7ab6fd87972e59f16312f451bce217a56/models) and [here](https://docs.px4.io/master/en/simulation/gazebo_vehicles.html), but almost all of them are deleted. 

You can specify other vehicles to keep by rebuilding `starling-sim-base-core` with the build arg `KEEP_PX4_VEHICLES` set to the vehicles you wish to keep. To rebuild you will need to clone the `ProjectStarling` repository, navigate to simulation and locally rebuild the correct controller:
```
docker build --build-args KEEP_PX4_VEHICLES="! -o -path iris_with_standoffs"
```

## Adding your own Models into the container

There will be many cases in which you will want to add extra models into your simulations. This might be to create an environment, or to add your own custom vehicles. For some exaples of projects which utilise custom models or worlds, please see the following:

- [FenswoodScenario](https://github.com/StarlingUAS/FenswoodScenario) where a number of custom models (downloaded from the gazebo repository) and a custom world is defined.
- [BRLFlightArenaGazebo](https://github.com/StarlingUAS/BRLFlightArenaGazebo) which is comprised of a custom world and a custom gimbal-tripod model built from scratch. 

### Background and `/ros.env.d`

Both the [base simulation container](../docker-images/sim-base-core.md), and the [base controller container](../docker-images/controller-base.md) both use a `/ros.env.d` mechanism. 

1. Both containers are set-up with a special folder in the root directory called `/ros.env.d`. 
2. When the container is started up, there is a special script that runs (called the *entrypoint* script) which looks at all of the folders within `/ros.env.d` and whether they have their own `setup.bash` script. 
3. If a `setup.bash` script exists, it will be [`source`'d](https://superuser.com/questions/176783/what-is-the-difference-between-executing-a-bash-script-vs-sourcing-it), i.e. script is run with any environment variables defined being exported to be used in later processes. 

By mounting external folders into the Dockerfile into `/ros.env.d`, we can add external and extra resources into a given controller or simulation container. 

### Setting up the project

The general format for a starling project with extra assets is the following: 

```
project/
├─ custom_models/
│  ├─ model_1/
│  │  ├─ setup.bash
│  │  ├─ models/
│  │  │  ├─ model_1/
│  │  │  |  ├─ meshes/ # optionally populated
│  │  │  │  ├─ model.config
│  │  ├─ model.sdf.xacro
│  ├─ model_2/
│  │  ├─ setup.bash
│  │  ├─ models/
│  │  │  ├─ model_2/
│  │  │  │  ├─ model.config
│  │  ├─ model.sdf.xacro
├─ custom_world/
│  ├─ world/
│  │  ├─ my_custom_world.world
│  ├─ models/
│  │  ├─ some_downloaded_model_asset/
│  ├─ setup.bash
├─ docker-compose.yaml
├─ system.launch.xml
├─ Makefile
├─ Dockerfile
```

This project seperates custom models and the custom worlds into two seperate folders. The `system.launch.xml` defines the roslaunch file being run by the container. The Dockerfile will then copy all of these elements into the container at build time.

### Custom Model

#### Specifying the model

As above, it is recommended that you have 3 items within the custom model folder. 

1. `setup.bash` which we will populate later
2. `model.sdf.xacro` defines the model as an sdf file with xacro argument replacement, I will explain what this means next
3. `models/` folder which creates a template of the file structure required by gazebo for any model.

Xacro is handy tool for specifying arguments to an sdf file in order to generate custom sdf files. This is needed as the sdf file which describes a model is usually hard coded to a particular vehicle instance. Xacro allows us to dynamically generate these files. 

An example model which uses the `model.sdf.xacro` is given here:
```xml
<?xml version='1.0'?>
<sdf version="1.6" xmlns:xacro='http://ros.org/wiki/xacro'>
  <xacro:property name="ros_namespace" value="$(arg ros_namespace)"/>
  <model name="iris_demo">
    <link name="tripod">
      <collision name="geom_1">
        <geometry>
          <box><size>0.5 0.5 1.5 0 0 0</size></box>
        </geometry>
      </collision>

      <!-- Keep collision box, but dont show it -->
      <visual name="tripod">
        <geometry>
          <box><size>0.5 0.5 1.5 0 0 0</size></box>
        </geometry>
        <material>
        <uri>file://media/materials/scripts/gazebo.material</uri>
        <script>Gazebo/Grey</script>
        </material>
      </visual>
    </link>

    <include>
      <uri>model://gimbal_small_2d</uri>
      <pose>0 0 1.0 1.57 0 1.57</pose>
    </include>

    <joint name="${ros_namespace}_gimbal_mount" type="revolute">
      <parent>tripod</parent>
      <child>gimbal_small_2d::base_link</child>
      <axis>
        <limit>
          <lower>0</lower>
          <upper>0</upper>
        </limit>
        <xyz>0 0 1</xyz>
        <use_parent_model_frame>true</use_parent_model_frame>
      </axis>
    </joint>

  </model>
</sdf>
```

This example describes the sdf file for a gimbal mounted on a simple tripod block. The xacro elements specify a 'property' of `ros_namespace`. When compiling with the xacro.py tool, it can generate a new sdf file with that property set to a value you want. 

You must then setup a model directory looking like the following (with optional meshes if using a model from the internet)
```
│  │  ├─ models/
│  │  │  ├─ model_1/
│  │  │  |  ├─ meshes/ # optionally populated
│  │  │  │  ├─ model.config
```

The exact contents of the model file follow gazebo guidelines. There is an overview [here](http://gazebosim.org/tutorials?tut=build_model). There are a number online as well. 

> Note that Xacro does not change the sdf or gazebo model syntax. It can be thought of as doing a copy paste to your existing file.

#### Building the model (`setup.bash`)

Now we have the model defined, we now want to ensure that the model can now be built/configured/compiled during the container runtime. This involves writing the `setup.bash` file. Usually there are 3 steps:

1. Making sure the script is being run from the correct directory, and that the relevant paths are set. 
2. Apply xacro with environment variables (given by the user or the container itself) and saving it in the model folder path (next to `model.config`)
3. Adding the model to the gazebo model path

Here is an example `setup.bash` for the gimbal example.

```bash
#!/bin/bash
SCRIPT_DIR="$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

XACRO_PATH=${SCRIPT_DIR}/model.sdf.xacro
MODEL_PATH=${SCRIPT_DIR}/models/gimbal_tripod/model.sdf

xacro ${XACRO_PATH} \
    ros_namespace:=${GIMBAL_NAMESPACE} \
    -o ${MODEL_PATH}

echo "Written Gimbal model to ${MODEL_PATH}"

export GAZEBO_MODEL_PATH="${SCRIPT_DIR}/models:${GAZEBO_MODEL_PATH}"
```

In particular the `xacro` command takes the `sdf.xacro` we created previously and creates a new version with the parameters realised - here `ros_namespace` is set to the environment variable `GIMBAL_NAMESPACE`. Environment variables are often defined in the Dockerfile or by the user. 

### Custom World

#### Specifying the world

A world can be defined by the worlds folder. Inside the worlds folder, there is the world file which describes the base world gazebo will generate. The world file itself is an sdf file with the `world` tag. Since the world is fixed, there is also no need for xacro (although you could use it if you wanted to).

```xml
<?xml version="1.0" ?>
<sdf version="1.4">
  <world name="flightarena">

    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <latitude_deg>51.4233628</latitude_deg>
      <longitude_deg>-2.671671</longitude_deg>
      <elevation>100.0</elevation>
    </spherical_coordinates>

    <include>
      <uri>model://sun</uri>
    </include>
    
    ... All the contents of the world ...

  </world>
</sdf>
```

An example can be found in these repositories [FenswoodScenario](https://github.com/StarlingUAS/FenswoodScenario/blob/master/fenswood/worlds/fenswood.world), [BRLFlightArenaGazebo](https://github.com/StarlingUAS/BRLFlightArenaGazebo/blob/master/flightarena/worlds/flightarena.world). 

The contents of the world file uses the same syntax as your normal model files. See the examples for adding primitive objects. For models, you will need to include the models inside this directory (and add that folder onto your path in the next step).
#### Building the model (`setup.bash`)

Similar to setup for the custom model, we need to define a setup for the custom world. For worlds, this usually just involves ensuring that the world and asscoiated models are on the gazebo paths:

```bash
SRC_PATH="$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# Add flightarena.world to the Gazebo resource path
export GAZEBO_RESOURCE_PATH=$SRC_PATH/worlds:${GAZEBO_RESOURCE_PATH}
echo "RESOURCE_PATH: $GAZEBO_RESOURCE_PATH"

# Add the 'grass_box' model to the Gazebo model path
export GAZEBO_MODEL_PATH=$SRC_PATH/models:${GAZEBO_MODEL_PATH}
echo "GAZEBO_MODEL_PATH: $GAZEBO_MODEL_PATH"

# Make the default media files work
cp -r /usr/share/gazebo-11/media $GZWEB_WS/http/client/assets/
(cd /root/gzweb/http/client/assets/media/materials/textures \
    && for f in *jpg; do convert $f ${f%.*}.png; done)
```

The final line is to ensure that all of the textures are compatible with the Gazebo web interface. 

### Building and Running the models

Now that all of the models and worlds have been created, we then need to either bundle it up, or inject it into the container so it can be included. 

There are two methods. Both of which are applicable depending on eventual usecase. 

1. Docker-Compose: The models/worlds can be mounted into the simulation container at runtime
2. Building a new Docker Container: If you know that you want to build upon the models or worlds into multiple differing applications, it's probably best to package it up and use as a dependency for the child projects. 

#### Docker-Compose

As mentioned earlier, as long as the folders have a `setup.bash` script inside, it is compatible with automation within `/ros.env.d`. Therefore you can mount your folders into the container as volumes within the docker-compose file:

```yaml
simhost:
    image: uobflightlabstarling/starling-sim-iris:${STARLING_RELEASE:-latest}
    volumes:
      - ./flightarena:/ros.env.d/02_flightarena
      - ./systems/models/gimbal_small_2d:/ros.env.d/03_gimbal_small_2d
      - ./systems/models/gimbal_tripod:/ros.env.d/04_gimbal_tripod
      - ./system.launch.xml:launch/system.launch.xml
    env:
      - GIMBAL_NAMESPACE=gimbal_1 
    command: ["ros2", "launch", "launch/system.launch.xml"]
    ports:
      - "8080:8080"
```

Where the syntax is `<local file>:<container file>`. Also note that inside `ros.env.d` we name the folders `XX_<name>` where `XX` is a two digit number describing the order of script loading from 0 being first to 99 being last. This is useful for dependencies.

#### DockerFile

The dockerfile is an alternative way where we explicitly copy the files into the newly built container. The following dockerfile does exactly this. 

```dockerfile
ARG VERSION=latest
ARG REGISTRY
FROM ${REGISTRY}uobflightlabstarling/starling-sim-iris:latest

# Copy in the xacro & bash file to setup the model
# Using ros.env.d automatic sourcing on entrypoint (see /simulator/base/core/Dockerfile)
COPY flightarena /ros.env.d/02_flightarena
COPY systems/models/gimbal_small_2d /ros.env.d/03_gimbal_small_2d
COPY systems/models/gimbal_tripod /ros.env.d/04_gimbal_tripod

# Build gimbal plugin with ROS2 on path and add plugin to path
COPY systems/gimbal_plugin /ros_ws/src/gimbal_plugin
RUN cd /ros_ws \
    && . /opt/ros/foxy/setup.sh \
    && export CMAKE_PREFIX_PATH=$AMENT_PREFIX_PATH:$CMAKE_PREFIX_PATH \
    && colcon build --cmake-force-configure \
    && rm -r build \
    && echo 'export GAZEBO_PLUGIN_PATH=/ros_ws/install/gimbal_plugin/lib:${GAZEBO_PLUGIN_PATH}' >> /ros.env.d/03_gimbal_small_2d/setup.bash

# Copy in the vehicle launch file
COPY system.launch.xml /ros_ws/launch/

WORKDIR /ros_ws

ENV GIMBAL_NAMESPACE gimbal_1

# Launch gazebo and spawn model
CMD [ "ros2", "launch", "launch/system.launch.xml" ]
```

Often you want to also setup the build process. See the examples for details! 