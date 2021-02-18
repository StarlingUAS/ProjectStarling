MAKEFILE_DIR:=$(shell dirname $(realpath $(firstword $(MAKEFILE_LIST))))

build:
	docker build -t starling-ardupilot-sitl $(MAKEFILE_DIR)
	docker tag starling-ardupilot-sitl uobflightlabstarling/starling-ardupilot-sitl

run: build
	docker run -it --rm --name sitl starling-ardupilot-sitl:latest

push: build
	docker push uobflightlabstarling/starling-ardupilot-sitl

.PHONY: build run push