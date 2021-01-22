


build:
	docker build -t mavros -f Dockerfile.mavros . 
	docker build -t controller -f Dockerfile.controller .

run_controller:
	docker run -it --rm --network="host" controller bash

run_controller_shell:
	docker exec -it 

run_mavros: 
	docker run -it --rm --network="host" mavros bash

save_controller:
	docker image save controller |gzip > controller.tar.gz