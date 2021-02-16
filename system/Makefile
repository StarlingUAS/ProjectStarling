MAKEFILE_DIR:=$(shell dirname $(realpath $(firstword $(MAKEFILE_LIST))))

all: mavros

mavros:
	docker build -t starling-mavros $(MAKEFILE_DIR)/mavros
	docker tag starling-mavros uobflightlabstarling/starling-mavros:latest

mavros_all_arch: 
	docker buildx build --push --platform linux/arm64,linux/amd64 --tag uobflightlabstarling/starling-mavros:latest $(MAKEFILE_DIR)/mavros 


.PHONY: mavros mavros_all_arch