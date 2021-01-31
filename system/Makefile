MAKEFILE_DIR:=$(shell dirname $(realpath $(firstword $(MAKEFILE_LIST))))

all: mavros

mavros:
	docker build -t starling-mavros $(MAKEFILE_DIR)/mavros

.PHONY: mavros