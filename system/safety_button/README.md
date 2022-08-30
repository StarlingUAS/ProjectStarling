# Safety Button for Flight Arena

This folder contains the files required to run the safety button within the flight arena.

It is constructed from a Teensy 3.2 running micro-ros with arduino. 

Micro-ros requires that the computer connected to it is running the [micro-ros-agent](https://hub.docker.com/r/microros/micro-ros-agent)

## Use

### Buttons
The following describes the functions:

1. **ESTOP**: The top toggle-able *'ESTOP'* red button will send `std_msgs/msg/Empty.msg` over the `/emergency_stop` topic continuously at 10hz
2. **GO**: The middle green button will send `std_msgs/msg/Empty.msg` over the `/mission_start` topic continuously at 10hz whilst pressed
3. **ABORT**: The bottom red button will send `std_msgs/msg/Empty.msg` over the `/mission_abort` topic continuously at 10hz whilst pressed

### LEDS
The following describes the 3 lights on the left hand side of the button

1. **RED LED**: `/emergency_stop` - Blinks at a maximum of 20hz when a topic is received.
2. **YELLOW LED**: `/mission_abort` - Blinks at a maximum of 20hz when a topic is received.
3. **GREEN LED**: `/mission_start` - Blinks at a maximum of 20hz when a topic is received.

**IF ALL LEDS ARE FLASHING (SLOW OR FAST), IT HAS LOST CONNECTION WITH THE MICRO_ROS AGENT RUNNING ON THE CONNECTED MACHINE. WAIT A MINUTE FOR RECONNECTION OR CHECK CONNECTION**

> Reconnection code comes from [here](https://github.com/micro-ROS/micro_ros_arduino/issues/400#issuecomment-903754040)

## Setup and Installation

## Button

The system was setup with instructions given here: https://manzurmurshid.medium.com/how-to-connect-teensy-3-2-with-micro-ros-and-ros2-foxy-6c8f99c9b66a

### Micro-Ros agent

The agent can most easily run using the following docker-container.
```
docker run -it --rm --net=host --privileged -v /dev:/dev microros/micro-ros-agent:foxy serial --dev /dev/ttyACM0
```

where the button is connected via USB on `/dev/ttyACM0`

Make sure to set the `ROS_DOMAIN_ID` to 2 when running the agent

### Setting Domain ID

As the Arduino runs its own DDS node, you have to set it's domain_id to match that of the rest of the ROS2 DDS network. This involves hardcoding it at build-time. This is achieved by setting the below node options.

```c++
// Set node options, set ros domain id to 2 corresponding with ROS_DOMAIN_ID
rcl_node_options_t node_ops = rcl_node_get_default_options();
node_ops.domain_id = 2;
```

