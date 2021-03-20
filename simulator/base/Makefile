MAKEFILE_DIR:=$(shell dirname $(realpath $(firstword $(MAKEFILE_LIST))))

all: core base-px4 px4-sitl

core:
	docker build -t starling-sim-base-core ${MAKEFILE_DIR}/core

px4-builder:
	docker build --target px4builder -t starling-px4-builder ${MAKEFILE_DIR}/px4

base-px4: px4-builder core
	docker build -t starling-sim-base-px4 ${MAKEFILE_DIR}/px4

px4-sitl: px4-builder
	docker build -t starling-sim-px4-sitl -f ${MAKEFILE_DIR}/px4/sitl.Dockerfile ${MAKEFILE_DIR}/px4

.PHONY: all core px4-builder base-px4 px4-sitl