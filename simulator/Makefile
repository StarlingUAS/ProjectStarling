MAKEFILE_DIR:=$(shell dirname $(realpath $(firstword $(MAKEFILE_LIST))))

all: clover2 ardupilot_sitl iris

base:
	$(MAKE) -C base

ardupilot_sitl:
	$(MAKE) -C ardupilot_ar

clover2: base
	docker build -t starling-sim-clover2 $(MAKEFILE_DIR)/vehicles/clover2

iris: base
	docker build -t starling-sim-iris $(MAKEFILE_DIR)/vehicles/iris

.PHONY: all base ardupilot_sitl clover2 iris