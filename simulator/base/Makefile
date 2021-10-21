MAKEFILE_DIR:=$(shell dirname $(realpath $(firstword $(MAKEFILE_LIST))))
BAKE_SCRIPT:=$(MAKEFILE_DIR)/../../buildtools/docker-bake.hcl
BUILDX_HOST_PLATFORM:=$(shell docker buildx inspect default | sed -nE 's/^Platforms: ([^,]*),.*$$/\1/p')
BAKE:=cd ../.. && docker buildx bake --builder default --load --set *.platform=$(BUILDX_HOST_PLATFORM) -f $(BAKE_SCRIPT)

all: core base-px4 px4-sitl ardupilot

core:
	$(BAKE) starling-sim-base-core

# Pseudotarget for PX4 group
px4: base-px4 px4-sitl

base-px4: core
	$(BAKE) starling-sim-base-px4

px4-sitl:
	$(BAKE) starling-sim-px4-sitl

# Pseudotarget for ArduPilot group
ardupilot: ardupilot-gazebo arducopter arduplane

ardupilot-gazebo: core
	$(BAKE) starling-sim-ardupilot-gazebo

arducopter:
	$(BAKE) starling-sim-ardupilot-copter

arduplane:
	$(BAKE) starling-sim-ardupilot-plane

.PHONY: all core px4 base-px4 px4-sitl ardupilot ardupilot-gazebo arducopter arduplane