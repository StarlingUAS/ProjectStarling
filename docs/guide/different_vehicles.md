# Different Vehicles in Simulation

With the way the system is setup, it can be quite simple to run another vehicle in simulation, this page shows two different methods of achieving this effect.

[TOC]

## Using Existing Models

There are a number of already loaded models into the SITL and Simulation containers. 

- For PX4 SITL container, the following models already exist: [github link](https://github.com/PX4/PX4-SITL_gazebo/tree/822050a7ab6fd87972e59f16312f451bce217a56/models) and [here](https://docs.px4.io/master/en/simulation/gazebo_vehicles.html)
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


## Adding your own Models into the container