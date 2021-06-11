MAKEFILE_DIR:=$(shell dirname $(realpath $(firstword $(MAKEFILE_LIST))))

all: takeoff_controller_python

takeoff_controller_python:
	docker build -t $@ $(MAKEFILE_DIR)

.PHONY: takeoff_controller_python all