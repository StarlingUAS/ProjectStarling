
all: ros_demo_node

ros_demo_node:
	docker build -t ros_demo -f Dockerfile.ros_demo_node .
	docker tag ros_demo mickeyli789/ros_demo

multi-platform:
	docker buildx build --push --platform linux/arm64,linux/amd64,linux/arm/v7 --tag mickeyli789/ros_demo:latest -f Dockerfile.ros_demo_node .

push:
	docker push mickeyli789/ros_demo

run:
	docker run -it --rm ros_demo