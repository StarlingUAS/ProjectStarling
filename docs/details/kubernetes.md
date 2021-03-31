# Kubernetes

[TOC]

## Overview


## Kubernetes in Starling


## Practical Details

### Root/Non-root access

By default the k3s installation creates a configuration file `k3s.yaml` which stores access tokens, the IP address of the master servers and various other information. By default this is stored in `/etc/rancher/k3s/k3s.yaml`. Any time `k3s` is invoked, i.e. any time you call `k3s kubectl ...`, this configuration file is accessed and read. However because the configuration file is stored in the system directory `/etc/`, by default any call to `k3s` would need `sudo`, i.e. `sudo k3s kubectl ...`. 

This can be avoided by moving the `k3s.yaml` file to your local user directory. The recommended location is `~/.kube/config/k3s.yaml`, but it can be put anyway. The only requirement is that the `KUBECONFIG` environment variable should be set to the location of the configuration file. This can be achieved by adding the following line somewhere into your `~/.bashrc` (or `~/.zshrc` if using zsh):

```bash
export KUBECONFIG=~/.kube/config/k3s.yaml
```
> **Note:** Do not forget to source your bashrc after making the change to see the change in your terminal `source ~/.bashrc`

Once this is set, any call of `k3s ...` will not require `sudo`. 

### Access Across Machines

If you are in the BRL Flight Arena, or you are running a multiple machine cluster, you may want to be able to run and test containers on a different physical machine to the machine running the cluster master node. As detailed in the above section on [non-root access](#rootnot-rootaccess), k3s generates the `k3s.yaml` configruation file. Importantly this file tells k3s what the ip of the master node is as shown in this example snippet of a k3s file:

```yaml
apiVersion: v1
clusters:
- cluster:
    certificate-authority-data: <long data string>
    server: https://127.0.0.1:6443 ### <---- This line! 
  name: default
contexts:
- context:
    cluster: default
    user: default
  name: default
current-context: default
kind: Config
preferences: {}
users:
- name: default
  user:
    client-certificate-data: <another long data string>
    client-key-data: < secret client key!>
```

To therefore access the master node (and the dashboard and kubectl etc) on another machine, you must.

1. Copy the `k3s.yaml` file off of the server onto your own machine. (For the BRL, see below)
2. In the `k3s.yaml` change the server ip from `127.0.0.1` to the ip address of the master machine. 
3. Ensure that either KUBECONFIG is set, or you have replaced the filee at `~/.kube/config/k3s.yaml` with your own modified one (may need sudo)

Once that is set up verify that you can get the dashboard access token and log in.

```bash
k3s kubectl -n kubernetes-dashboard describe secret admin-user-token | grep ^token
```

Also then verify that you can run any of the kubernetes deployments, and that they will be deployed to the master cluster (and not your own).

> **Note:** Even if the cluster is not running on your local machine, you will still need to install the k3s binaries. The command run by `run_k3s.sh` or `./scripts/start_k3s.sh` both download and then automatically start k3s in the background of your machine. To stop it from starting, pass the `--do-not-start` option to either command. If you have already have k3s running, run `k3s-killall.sh` which will stop all k3s processes without uninstalling it entirely (so you still get access to the k3s command line tools). 

In the BRL, to get the k3s.yaml file, you should simply be able to scp (ssh copy) the file from the master machine `~/.kube/config/k3s.yaml` to your local machine, and change the IP address.

## Configuration Files



