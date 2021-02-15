.PHONY: all ros_debug_node multi-platform push run_dev run

all: ros_debug_node

ros_debug_node:
	docker build -t ros_debug .
	docker tag ros_debug mickeyli789/ros_debug

multi-platform: ros_debug_node
	docker buildx build --push --platform linux/arm64,linux/amd64 --tag mickeyli789/ros_debug:latest .

push:
	docker push mickeyli789/ros_debug

run_dev:
	docker run -it --rm -v $(shell pwd)/workspace:/home/ctrl_user/workspace ros_debug /bin/bash

run:
	docker run -it --rm --network="host" ros_debug 