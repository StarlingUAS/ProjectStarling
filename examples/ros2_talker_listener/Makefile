
containername=starling-controller
tag=latest
name=controller
network=bridge
flags=-it --rm --network "$(network)"

build:
	docker build -t mavros -f ../../controller/Dockerfile.mavros . 
	docker tag mavros starling-mavros:latest
	docker build -t controller -f Dockerfile.controller .
	docker tag controller $(containername):$(tag)
	docker build -t ardupilot-sitl-docker:starling ../../simulator/ardupilot_ar

run_controller:
	docker run -it --rm --name controller --network="bridge" controller bash

run_listener_only:
	docker run $(flags) --name ctrl_listener controller ./launch_listener.sh

run_talker_only:
	docker run $(flags) --name ctrl_talker controller ./launch_talker.sh

run_controller_shell:
	docker exec -it controller

dev_controller:
	docker run -it --rm --network="bridge" --name controllerdev -v $(shell pwd)/workspace:/home/ctrl_user/workspace controller bash

dev_controller_shell:
	docker exec -it controllerdev

run_mavros:
	docker run -it --rm --name mvr mavros

run_mavros_host: 
	docker run -it --rm --name mvr --network="host" mavros

attach_mavros_shell:
	docker exec -it mvr bash

save_controller:
	docker image save controller |gzip > controller.tar.gz