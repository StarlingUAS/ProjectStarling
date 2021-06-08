MAKEFILE_DIR:=$(shell dirname $(realpath $(firstword $(MAKEFILE_LIST))))

all: clover

clover-local:
	docker build -t starling-clover ${MAKEFILE_DIR}/clover

clover: 
	docker buildx build --no-cache --push --platform linux/arm64,linux/amd64 --tag mickeyli789/starling-clover:latest $(MAKEFILE_DIR)/clover 

.PHONY: clover-local clover

