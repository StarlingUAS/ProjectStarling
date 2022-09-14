# Setting up a new Vehicles for the flight arena

[TOC]

## Vehicle Specification

Starling can be made compatible with almost all vehicles both ground and aerial. In the majority of cases, a vehicle is valid if it has enough compute to run docker containers and connect to the network. The recommended minimum is a Raspberry Pi 4 with 2Gb RAM with WiFi connectivity. The compute should be sufficient such that all the containers can be run safely without saturating the CPU, Memory or Network Bandwidth. 

### Aerial Vehicles

The current system is designed with a PX4/Ardupilot compatible flight control (such as a Pixhawk or PixRacer) running in tandem with a companion computer such as a Raspberry Pi. 

The current risk assessment covers multi-vehicle flight for vehicles < 1kg maximum take off mass. 

## Adding a new vehicle to Starling

> Note: You will need SSH access to the flight arena server. 

### Companion Computer Boot Image

We have created a custom image which contains the minimum general elements to be compatible with Starling. The image has been generated using the code in the [starling-64-mod branch of UobFlightLab/pi-gen](https://github.com/UoBFlightLab/pi-gen/tree/starling-64-mod). We have released a precompiled image file which can be flashed onto a USB or SD-card and booted from using a tool such as the [Raspberry Pi Imager](https://www.raspberrypi.org/software/)

The image is based on Ubuntu 20.04 for ARM chips (Raspberry Pi), and has the following elements: 

* Password-less sudo
* Automatic connectivity to the Flight Arena LAN
* k3s airgapped installation files placed into root with `k3s_install.sh` executable
* GPIO drivers installed 
* To ensure connectivity with the pixhawk, [`99-px4fmu.rules`](https://github.com/CopterExpress/clover/blob/master/clover/udev/99-px4fmu.rules) has been added to `/lib/udev/rules.d` to ensure the pixhwak is connected via `/dev/px4fmu`.
* `Chrony` library has been installed and that `/etc/chonry/chonry.conf` is configured to use the flight arena server at `192.168.10.80` [(see the BRL flight arena doc)](../flight_arena#time-synchronisation)
* The Docker Daemon registry mirrors has been set via `/etc/docker/daemon.json` to use the flight arena server at `192.168.10.80:5000` to ensure the drone can pull docker images from the internet. [(see the BRL flight arena doc)](../flight_arena#docker-local-registry-as-a-pass-through-cache)

**Connecting Pixhawk via UART Serial** (Added 14/09/2022)

By default, the pixhawk is connected via the USB ports. However the pixhawk can also be connected directly by UART Serial via the serial Rx/Tx (14/15) pi GPIO pins. To enable this to work [reference](https://raspberrypi.stackexchange.com/questions/114366/rpi4-serial-port-not-working-on-either-raspberry-os-or-ubuntu):

* add `enable_uart=1` to `/boot/config.txt`
* disable the serial console: `sudo systemctl stop serial-getty@ttyS0.service && sudo systemctl disable serial-getty@ttyS0.service`

> This can also done using `raspi-config` as specified in [ardupilot docs](https://ardupilot.org/dev/docs/raspberry-pi-via-mavlink.html)

Then to set up the connection so that it is the same as previous, you will need to add the following line to the bottom of `/lib/udev/rules.d/99-px4fmu.rules`. This assumes that the serial connection exists through `/dev/ttyS0`. This will symlink it to `/dev/px4fmu` as before. 

```
# Serial Connection
KERNEL=="ttyS0", SYMLINK+="px4fmu"
```

Note also that you will need to set the correct parameters on the pixhawk itself. [See this example of PX4](https://docs.px4.io/main/en/peripherals/mavlink_peripherals.html#example)

### Vehicle Specific Setup

A vehicle ID number from 10 to 255 must be chosen. IDs below 10 are reserved. It must be checked that IDs do not clash with existing vehicles (i.e. the Coex Clover drones take ids 11 - 14). 

If the vehicle is going to be used with PX4 or Ardupilot, the flight controller system id should be chaned to the vehicle ID. e.g. setting `MAV_SYS_ID` in QGroundControl for Px4 based systems.

If the vehicle is going to be used with the [Mavros Container](../starling-mavros#running-on-a-vehicle), a `/etc/starling/vehicle.config` file is required. An example is the following:
```
VEHICLE_FCU_URL=serial:///dev/px4fmu:115200
VEHICLE_FIRMWARE=px4
VEHICLE_MAVLINK_SYSID=23
VEHICLE_NAME=clover23
```

### Adding vehicle as a node. 

The vehicle can now be added as a node to the cluster. 

1. Ensure that the vehicle is only connected to the LAN either by WIFI or Ethernet. 
2. Identify the IP address of the vehicle by going to the DHCP server on `192.168.10.252`, or other means
3. SSH into the flying server and `cd` into the `ProjectStarling/scripts` directory
4. Run the agent setup script `start_k3s_agent.sh`. This will ssh into the vehicle and run the setup script with the keys held by the server. [See this doc](../kubernetes-deployment#pi-drone-agent) for more details.

    ```./start_k3s_agent.sh <remote username> <remote ip address> <node name>```

    e.g. ```./start_k8_agent.sh ubuntu 192.168.0.110 clover1```

5. Add the following tags to the node either through the edit functionality in the kubernetes dashboard (the pencil in the upper right corner) or manually through the command line. The tag keys must be fixed, but the values can be varied depending on application. These are required for automatic deployment of containers.

```
starling.dev/type: vehicle
starling.dev/vehicle-class: rotary
starling.dev/vehicle-type: clover
```

The vehicle should now be visible under `nodes` on the Kubernetes dashboard.