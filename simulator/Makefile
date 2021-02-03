MAKEFILE_DIR:=$(shell dirname $(realpath $(firstword $(MAKEFILE_LIST))))

all: clover2 ardupilot_ar

base:
	$(MAKE) -C base

ardupilot_ar:
	$(MAKE) -C ardupilot_ar

clover2: base
	docker build -t starling-sim-clover2 $(MAKEFILE_DIR)/vehicles/clover2

.PHONY: all base clover2 ardupilot_ar