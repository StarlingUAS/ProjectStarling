MAKEFILE_DIR:=$(shell dirname $(realpath $(firstword $(MAKEFILE_LIST))))

all: mavros

mavros:
	docker build -t starling-mavros $(MAKEFILE_DIR)/mavros

ui:
	docker build -t starling-ui $(MAKEFILE_DIR)/ui

.PHONY: mavros ui all