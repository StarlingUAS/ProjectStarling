MAKEFILE_DIR:=$(shell dirname $(realpath $(firstword $(MAKEFILE_LIST))))
BAKE_SCRIPT:=$(MAKEFILE_DIR)/../buildtools/docker-bake.hcl
BUILDX_HOST_PLATFORM:=$(shell docker buildx inspect default | sed -nE 's/^Platforms: ([^,]*),.*$$/\1/p')
BAKE:=cd .. && docker buildx bake --builder default --load --set *.platform=$(BUILDX_HOST_PLATFORM) -f $(BAKE_SCRIPT)

all: clover2 iris iris-ap

px4:
	$(MAKE) -C base px4

ardupilot:
	$(MAKE) -C base ardupilot

clover2: px4
	docker build -t starling-sim-clover2 $(MAKEFILE_DIR)/vehicles/clover2

iris: px4
	$(BAKE) starling-sim-iris

rover: px4
	$(BAKE) starling-sim-rover

iris-ap: ardupilot
	$(BAKE) starling-sim-iris-ap

.PHONY: all px4 ardupilot clover2 iris iris-ap
