MAKEFILE_DIR:=$(shell dirname $(realpath $(firstword $(MAKEFILE_LIST))))
BAKE_SCRIPT:=$(MAKEFILE_DIR)/../buildtools/docker-bake.hcl
BUILDX_HOST_PLATFORM:=$(shell docker buildx inspect default | sed -nE 's/^Platforms: ([^,]*),.*$$/\1/p')
BAKE:=cd $(MAKEFILE_DIR)/.. && docker buildx bake --builder default --load --set *.platform=$(BUILDX_HOST_PLATFORM) -f $(BAKE_SCRIPT)

all: controller-base mavros ros-web-bridge vicon mavp2p

controller-base:
	$(BAKE) starling-controller-base

mavros:
	$(BAKE) starling-mavros

mavros_all_arch: 
	cd $(MAKEFILE_DIR)/.. && docker buildx bake --builder default --load -f $(BAKE_SCRIPT)

ros-web-bridge:
	$(BAKE) ros-web-bridge

mavp2p:
	$(BAKE) mavp2p

vicon:
	$(BAKE) starling-vicon

.PHONY: mavros mavros_all_arch controller-base ros-web-bridge vicon mavp2p all
