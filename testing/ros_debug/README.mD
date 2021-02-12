# Ros Debug Kubernetes Testing Nodes 

Contains a ros foxy image with a custom ros package `ros_debug_nodes` inside the workspace. 

The `ros_debug_nodes` package contains one node `talker` which 
1. Identifies the topics it can see on the ros network
2. Publishes these in a list along with a random name corresponding to that particular node. 

Also contains an example kubernetes file which runs multiple instances of these ros nodes on all k8nodes in the cluster. (Currently set to raspi1, clover1 and anything which matches amd64)

This is also an example of a ros2/ docker development environment where the 'workspace' directory is copied into the image and colcon built is run. However for development you can run `make run_dev` which also binds the workspace into the container, allowing you to make changes to the code natively, but then colcon build the code within the contianer itself. 