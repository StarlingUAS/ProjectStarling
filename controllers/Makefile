MAKEFILE_DIR:=$(shell dirname $(realpath $(firstword $(MAKEFILE_LIST))))

all: example_controller_python

example_controller_python:
	docker build -t $@ $(MAKEFILE_DIR)/$@
	docker tag $@ uobflightlabstarling/$@

.PHONY: example_controller_python all