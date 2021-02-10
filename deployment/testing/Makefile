
all: ros_demo_node

ros_demo_node:
	docker build -t ros_demo -f Dockerfile.ros_demo_node .
	docker tag ros_demo mickeyli789/ros_demo
	docker push mickeyli789/ros_demo

run:
	docker run -it --rm ros_demo