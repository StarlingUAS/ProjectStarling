


build:
	docker build -t mavros -f ../../controller/Dockerfile.mavros . 
	docker build -t controller -f Dockerfile.controller .

run_controller:
	docker run -it --rm --name controller --network="host" controller bash

run_controller_shell:
	docker exec -it controller

dev_controller:
	docker run -it --rm --network="host" --name controllerdev -v $(shell pwd)/workspace:/home/ctrl_user/workspace controller bash

dev_controller_shell:
	docker exec -it controllerdev

run_mavros: 
	docker run -it --rm --network="host" mavros bash

save_controller:
	docker image save controller |gzip > controller.tar.gz