# Flight Arena Architecture and Set Up

## Contents
[TOC]

## Architecture 

## Flight Arena Details

### Server Setup

For setting up a "master" server. All that should be needed is a Docker installation. Following
[Docker's instructions](https://docs.docker.com/engine/install/ubuntu/) is sufficient.

Then setup a Docker registry image to run as a "pull-through cache", again following
[Docker's instructions](https://docs.docker.com/registry/recipes/mirror/). Sufficient is to run:

```sh
docker run -d -p 5000:5000 --restart=always --name registry_mirror \
           -e REGISTRY_PROXY_REMOTEURL='https://registry-1.docker.io' \
           -e REGISTRY_DELETE_ENABLED=true \
           registry
```

To check on the status of the registry, use:

```sh
docker logs registry_mirror
```

### Time Synchronisation

When running the Vicon node on a separate PC, the clock of the onboard PC needs to be closely synched to allow the position estimates to be used by PX4. You can do this with `chrony`.

#### Installation

One liner if package is available:

```sh
sudo apt install chrony
```

#### Config

`chrony`'s configuration is in `/etc/chrony/chrony.conf`. The configuration needed depends on which side of the system is being configured.

##### Vehicle side

On the vehicle, `chrony` needs to be configured to add a time server on the local network. This can be done by adding the below to the config file:

```
# Use local time source
server ${GROUND_PC_IP} iburst prefer
```

##### Ground side

On the ground, `chrony` is configured to accept connections from clients on the local network:

```
# Allow clients to connect to this computer
allow ${LOCAL_SUBNET_PREFIX} # e.g. 192.168.10
bindaddress ${GROUND_PC_IP}
local stratum 10
```

#### Restart

Once the edits to the config file have been made, restart `chrony` through `systemd`:

```sh
sudo systemctl restart chrony
```

#### Troubleshooting

`chronyc sources` will show the current status of the computer's time sync.

`chronyc makestep` should force the computer to synchronise with the server.
