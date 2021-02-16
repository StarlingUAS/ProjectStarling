.PHONY: all ros_demo_node multi-platform push run_dev run

all: ros_demo_node

ros_demo_node:
	docker build -t ros_demo -f Dockerfile.ros_demo_node .
	docker tag ros_demo uobflightlabstarling/ros_demo

multi-platform: 
	docker buildx build --push --platform linux/arm64,linux/amd64 --tag uobflightlabstarling/ros_demo:latest -f Dockerfile.ros_demo_node .

push: ros_demo_node
	docker push uobflightlabstarling/ros_demo

run: ros_demo_node
	docker run -it --rm ros_demo