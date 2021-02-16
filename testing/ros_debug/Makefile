.PHONY: all ros_debug_node multi-platform push run_dev run

all: ros_debug_node

ros_debug_node:
	docker build -t ros_debug .
	docker tag ros_debug uobflightlabstarling/ros_debug

multi-platform: 
	docker buildx build --push --platform linux/arm64,linux/amd64 --tag uobflightlabstarling/ros_debug:latest .

push: ros_debug_node
	docker push uobflightlabstarling/ros_debug

run_dev: ros_debug_node
	docker run -it --rm -v $(shell pwd)/workspace:/home/ctrl_user/workspace ros_debug /bin/bash

run: ros_debug_node
	docker run -it --rm --network="host" ros_debug 