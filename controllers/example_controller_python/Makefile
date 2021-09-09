MAKEFILE_DIR:=$(shell dirname $(realpath $(firstword $(MAKEFILE_LIST))))

all: example_controller_python

example_controller_python:
	$(MAKE) -C $@

.PHONY: example_controller_python all