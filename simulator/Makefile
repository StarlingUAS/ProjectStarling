MAKEFILE_DIR:=$(shell dirname $(realpath $(firstword $(MAKEFILE_LIST))))

all: clover2 iris iris-ap

base:
	$(MAKE) -C base

clover2: base
	docker build -t starling-sim-clover2 $(MAKEFILE_DIR)/vehicles/clover2

iris: base
	docker build -t starling-sim-iris $(MAKEFILE_DIR)/vehicles/iris

iris-ap: base
	docker build -t starling-sim-iris-ap $(MAKEFILE_DIR)/vehicles/iris-ap

.PHONY: all base clover2 iris iris-ap
