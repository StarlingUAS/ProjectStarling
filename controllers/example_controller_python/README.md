# Example Python Controller

This is a very basic "controller" written in Python, using ROS2 and MAVROS to
control a PX4-based vehicle. It commands the vehicle to fly in a 1m radius
circle at a height of 1m.

There are a few phases that the controller goes through:
 1. #### Initialisation
    
    In the initialisation phase, the controller sends a stream of setpoints to
    the vehicle while it waits for a valid position. This is needed as PX4 will
    not switch into `OFFBOARD` mode without a valid setpoint stream. In this
    phase, the controller also initialises the initial position of the vehicle.

 1. #### Mode switching
 
    PX4 follows various types of setpoints when in `OFFBOARD` mode. The
    controller uses a ROS service to command the vehicle to switch into this
    mode. It will retry the mode switch once per second until it succeeds.

 1. #### Arming

    With the vehicle in the correct mode, the controller uses another ROS
    service to command the vehicle to arm. Once the vehicle is armed, the
    rotors begin to spin and the vehicle will begin trying to reach the
    setpoints it is provided with. Again the controller attempts this once
    per second until success.

 1. #### Takeoff

    Once the vehicle is armed and in `OFFBOARD` mode, the flight can begin. The
    controller uses the initial position it recorded earlier and begins to
    generate a setpoint that rises above this. Once the setpoint is at the
    desired takeoff altitude, the controller waits for the actual vehicle
    position to reach some predetermined value before it considers the takeoff
    to be complete.

 1. #### Flight

    With takeoff complete, the controller begins sending setpoints for the
    vehicle. In this case, these follow a circle around the local coordinate
    system origin.

## Notes

### Coordinate Systems

ROS and MAVROS use Front-Left-Up (FLU) coordinate systems while PX4 uses a
Front-Right-Down (FRD) coordinate system. Coordinates and vectors transferred
between the two systems are automatically transformed to the appropriate
convention by MAVROS. You should provide MAVROS with setpoints in FLU and
expect position data from it in FLU.

Another aspect to be aware of is PX4's treatment of the local coordinate
system. Generally, this coordinate system will be centred where the vehicle
was powered up. However, internal estimates are often poor during ground
handling, so there may be a signficant offset from your expectation.
Coordinating the local coordinate systems of multiple vehicles can become
complex.

If external local positioning information is provided, for example from Vicon,
PX4's coordinate system will share the same origin. This makes dealing with
multiple vehicles muc easier.

PX4's global coordinate system as exposed by MAVROS comes in two forms.
The first is a PoseStamped message, which is relative to either a provided map
origin, the home position, or the first GNSS fix received. The other is a full
geographic coordinate formed of latitude, longitude and altitude as part of a
NavSatFix message.

### ArduPilot Compataility

The controller should be mostly compatible with ArduPilot. The main difference
is the name that ArduPilot gives to the mode needed to obey setpoints. The
closest equivalent is ArduPilot's "GUIDED" mode. The controller has not been
tested with ArduPilot yet.

## ROS Specifics

There are a few ROS specifics that may look a little strange to the
uninitiated. The first is the blank file under `resource/`. This file is in
fact quite critical. It is used by the ROS index to let it know that this
package has been installed. The empty `__init__.py` file is there for similar
mysterious reasons. The second oddity is the `setup.cfg` file. This is again
a ROS specific configuration ensuring that things get installed into the
correct places.

`package.xml` provides ROS with information about the package. You should list
any dependencies your controller has here. For almost all of the controllers,
you will want to list `mavros_msgs` as a dependency. However, note that this
package is not yet available from the package managers for ROS2. Luckily
we prepared an image for you earlier with it already installed. A lot of
information from here then has to be duplicated into the `setup.py` script. If
only there was a way to automate the building of build scripts...
