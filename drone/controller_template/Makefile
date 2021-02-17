MAKEFILE_DIR:=$(shell dirname $(realpath $(firstword $(MAKEFILE_LIST))))

all: controller

controller:
	docker build -t starling-controller -f $(MAKEFILE_DIR)/Dockerfile.controller $(MAKEFILE_DIR)

.PHONY: controller