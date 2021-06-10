# Vehicle Setup

[TOC]

## Kubernetes Configuration

The implementation used in Starling, `k3s`, can be configured with a YAML file stored at:
`/etc/rancher/k3s/config.yaml`. This file will be read by the agent at start. Here, it is used to
add taints and labels to the Kubernetes node to allow control over where workloads are scheduled. An
example configuration from a Clover is given below:

```yaml
node-taint:
  - "starling.dev/type=vehicle:NoSchedule"
node-label:
  - "starling.dev/type=vehicle"
  - "starling.dev/vehicle-class=rotary"
  - "starling.dev/vehicle-type=clover"
```

In Kubernetes, a taint is applied to a node and is used to prevent workloads from being assigned to
it unless they explicitly "tolerate the taint" through a toleration in the deployment file. Here we
use it for ensuring that only workloads that are explicity allowed on a vehicle will be scheduled
on one. Note that a label with the same content still needs to be added if that is to be used as a
`nodeSelector`.

Kubernetes scheduling can also be controlled by selectors and affinities. A selector is a hard
requirement for a particular label on a node and an affinity is a more flexible version. The labels
applied to the Clover node above describe its overall class (rotary wing) and the specific vehicle
type (clover). These allow workloads to be scheduled to run only on particular vehicles.

### Local registry

If you're running a local registry, remember to add to `/etc/docker/daemon.json` to include the new
mirror:
```json
{
    "registry-mirrors": ["http://${REGISTRY_IP}:5000"]
}
```
## Time synchronisation

If you're running a separate time server, you'll also need to configure `chrony` on the vehicle.
This can be done by adding the below to the config file at `/etc/chrony/chrony.conf`.

```
# Use local time source
server ${GROUND_PC_IP} iburst prefer
```
