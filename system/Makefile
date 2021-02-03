MAKEFILE_DIR:=$(shell dirname $(realpath $(firstword $(MAKEFILE_LIST))))

all: mavros controller-base

controller-base:
	docker build -t starling-controller-base $(MAKEFILE_DIR)/controller-base

mavros:
	docker build -t starling-mavros $(MAKEFILE_DIR)/mavros

.PHONY: mavros controller-base all