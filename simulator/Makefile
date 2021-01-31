MAKEFILE_DIR:=$(shell dirname $(realpath $(firstword $(MAKEFILE_LIST))))

all: clover2

base:
	$(MAKE) -C base

clover2: base
	docker build -t starling-sim-clover2 $(MAKEFILE_DIR)/vehicles/clover2

.PHONY: all base clover2