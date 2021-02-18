# Testing

This directory contains a few projects used for system testing

## [Ros Debug](ros_debug/README.md)

Contains a ros foxy image with a custom ros package `ros_debug_nodes` inside the workspace. 

The `ros_debug_nodes` package contains one node `talker` which 
1. Identifies the topics it can see on the ros network
2. Publishes these in a list along with a random name corresponding to that particular node. 

Also contains an example kubernetes file which runs multiple instances of these ros nodes on all k8nodes in the cluster. (Currently set to raspi1, clover1 and anything which matches amd64)

This is also an example of a ros2/ docker development environment where the 'workspace' directory is copied into the image and colcon built is run. However for development you can run `make run_dev` which also binds the workspace into the container, allowing you to make changes to the code natively, but then colcon build the code within the contianer itself. 

To run first start up the k3s kubernetes cluster ([follow instructions here](../deployment/README.md)), then apply the configuration:
```bash
sudo k3s kubectl apply -f ros_debug/kubernetes.yaml
```
replace `apply` with `delete` to remove the node

### [Ros2 Demo talker/ listener](ros_demo/README.md)

This is in testing/ros_demo

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