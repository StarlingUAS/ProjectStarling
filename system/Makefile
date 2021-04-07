MAKEFILE_DIR:=$(shell dirname $(realpath $(firstword $(MAKEFILE_LIST))))

all: mavros controller-base ui discovery

controller-base:
	docker build -t starling-controller-base $(MAKEFILE_DIR)/controller-base

mavros:
	docker build -t starling-mavros $(MAKEFILE_DIR)/mavros

mavros_all_arch: 
	docker buildx build --push --platform linux/arm64,linux/amd64 --tag uobflightlabstarling/starling-mavros:latest $(MAKEFILE_DIR)/mavros 

ui:
	docker build -t starling-ui $(MAKEFILE_DIR)/ui

discovery:
	docker build -t starling-discovery $(MAKEFILE_DIR)/discovery

.PHONY: mavros mavros_all_arch controller-base ui discovery all
