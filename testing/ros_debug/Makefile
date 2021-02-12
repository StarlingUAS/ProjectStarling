all: ros_debug_node

ros_debug_node:
	docker build -t ros_debug .
	docker tag ros_demo mickeyli789/ros_debug

multi-platform:
	docker buildx build --push --platform linux/arm64,linux/amd64,linux/arm/v7 --tag mickeyli789/ros_debug:latest .

push:
	docker push mickeyli789/ros_debug

run_dev:
	docker run -it --rm -v $(shell pwd)/workspace:/home/ctrl_user/ ros_debug bash

run:
	docker run -it --rm ros_debug