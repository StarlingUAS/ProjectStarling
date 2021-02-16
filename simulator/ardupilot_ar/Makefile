MAKEFILE_DIR:=$(shell dirname $(realpath $(firstword $(MAKEFILE_LIST))))

build:
	docker build -t starling-ardupilot-sitl $(MAKEFILE_DIR)

# run:
# 	docker run -it --rm --name sitl starling-ardupilot-sitl:latest