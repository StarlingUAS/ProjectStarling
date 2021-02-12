# Deployment with kubernetes (k8)

This system is intended to run as a cloud platform. We utilse [k3s](https://rancher.com/docs/k3s/latest/en/quick-start/) as our kubernetes manager. 

Key concepts are as follows:
- **pods** are a k8 concept. A pod contains one or more containers and has its own ip address. Containers within a pod communicate over localhost
- **node** is the machine (physical e.g. pi or virtual machine) upon which pods are run. (Separate from 'ros2 nodes' or 'ros nodes')
- **cni** container networking interface (default is flannel for k3s) is the underlying networking for all containers
- **dds/ fast-rtps** Is the default communications middleware for ros2 comms.

Refer to the kubernetes notes with the onenote notebook for more usage information.

## Installation instructions

Install k3s using the install script, this will fetch k3s and run the kubernetes master node immediately:
```curl -sfL https://get.k3s.io | INSTALL_K3S_EXEC="--docker" sh -```

> Once installed, the equivalent instruction is `sudo k3s server --docker`

For the raspberry pi, ensure docker is installed, and then these instructions are the same on the raspberry pi (64 bit os).

For testing purposes (inside testing dir), the containers have already been built for both amd64 and arm64 and uploaded onto hub.docker: [mickeyli789/ros_demo](https://hub.docker.com/r/mickeyli789/ros_demo).

Also recommended you alias kubectl (kubernetes cli) in your bashrc
```alias kubectl='sudo k3s kubectl`

## Running instructions

### Laptop

To run the master kubernetes server using docker (instead of containerd if you need access to local images). In one terminal run:
```sudo k3s server --docker```

This will open up a server with entrypoint on 0.0.0.0:6443. 

### Pi
First SSH onto the pi

First ensure you run `k3s-killall.sh` to make sure there is no master server running as you only want `k3s agent` to run.

The K3S_TOKEN is the contents of the file `/var/lib/rancher/k3s/server/node-token`

```bash
K3S_TOKEN=<contents of the file /var/lib/rancher/k3s/server/node-token>
#e.g. K3S_TOKEN=K103b62838822f40f3e41j51f10cb127236f2c3014c120ede19263da9f33fbfc859::server:2dcbb32a4cad16e20d714d88dbce4af8
K3S_SERVER=https://<Your main machine ip address>:6443
K3S_NODE_NAME=clover1

echo "Killing all k3s services and instances first"
k3s-killall.sh

echo "Starting k3s agent only"
sudo k3s agent -t ${K3S_TOKEN} -s ${K3S_SERVER} --node-name ${K3S_NODE_NAME}
```
The Pi should now be setup

Consider running the above using screen or somehow in the background just in case your ssh connection is unstable or you want to close it. 

### Post actions
#### Dashboard
[See the original site for more details](https://kubernetes.io/docs/tasks/access-application-cluster/web-ui-dashboard/)
Back on the laptop, open up a new terminal then in this directory, run:

```bash
sudo kubectl apply -f k3.dashboard.yaml
sudo kubectl proxy # Open up connections to host
```
Kubectl will make Dashboard available at http://localhost:8001/api/v1/namespaces/kubernetes-dashboard/services/https:kubernetes-dashboard:/proxy/.

For first time setup, you will need to create an access token, refer to this article: https://www.replex.io/blog/how-to-install-access-and-add-heapster-metrics-to-the-kubernetes-dashboard 

```bash
kubectl create serviceaccount dashboard-admin-sa
kubectl create clusterrolebinding dashboard-admin-sa  --clusterrole=cluster-admin --serviceaccount=default:dashboard-admin-sa
kubectl get secrets
kubectl describe secret dashboard-admin-sa-token-kw7vn
```
The final command should display the token. Copy and paste that into the browser, and you should be in. 

In the dashboard, they key views are **nodes** and **pods** (within the default namespace).
Once a pod is running, there are options to view the logs and execute (exec) directly into the containers.

> Note: if you have reinstalled k3s dashboard and get the error `square/go-jose: error in cryptographic primitive`, you have a stale cookie!

## Running the test cases

### Ros2 talker/ listener

For the testing so far, we have 3 pods inside the [testing](testing) directory.
1. `k8.listener.yaml` runs a pod called `demo-listener` containing a ros2 listener node and a network toolbox. It will run on any node.
2. `k8.talkerlistener.yaml` runs a pod called `demo-talkerlistener` containg a ros2 listener node, a ros2 talker node an a network toolbox. Using the `nodeSelector` option, this pod will only run on arm64 nodes (i.e. nodes with the label `kubernetes.io/arch`)
3. `k8.talker.yaml` runs a pod called `demo-talker` containing a ros2 talker node. Will run on any node.

To 'deploy' these pods onto kubernetes, open another terminal and run:
```kubectl apply -f <file>.yaml```
These can also be strung together, for example
```kubectl apply -f k8.talkerlistener.yaml -f k8.listener.yaml```
Will run both the talker listener node on the pi and the listner node on the master. 

Once applied, both nodes should be able to be seen in the Pod section of the dashboard. Clicking on the pod names will reveale more information including ip address, status etc. The top right should have some buttons to see pod logs and also execute into the pod. Clicking on the logs (or exec), you can switch between the containers within the pod with a drop down menu above the logs. 

Replacing `apply` with `delete` will also delete the pods. If you want the pods to exit immediately, add the following flags onto the end
```kubectl delete -f k8.talkerlistener.yaml -f k8.listener.yaml --wait=false --grace-period=0 --force```

#### Expected outcome

Running the talker listener on the pi and the listener on the master node, I would expect that both listener nodes will be able to hear messages from the talker. 

#### Current Situation

Running both listener and talker on the same node (whether in separate pods or the same pod) runs fine mostly. 

Running the talker listener on the pi and the listener on the master node, either the listener on the pi hears, or the listener on the master hears and very rarely they do sometimes both hear at the same time. 

I susepct that there may be some race condition type effect. If only the talkerlistener is run, sometimes not observing or interacting with the logs for some time will cause the chatter to come through. Other times, the dashboard log may show that nothing has been received, however exec'ing into the listener and manually either running (after sourcing `ros_entrypoint.sh`) `ros2 topic echo /chatter` or `ros2 run demo_nodes_py listener` will produce an output. 
