MAKEFILE_DIR:=$(shell dirname $(realpath $(firstword $(MAKEFILE_LIST))))
all: simulator system controllers

simulator:
	$(MAKE) -C simulator

system:
	$(MAKE) -C system

controllers:
	$(MAKE) -C controllers

docs:
	cp ${MAKEFILE_DIR}/documentation/README.md ${MAKEFILE_DIR}/README.md
	mkdocs build

serve-docs:
	mkdocs serve -a 0.0.0.0:8000

run:
	docker-compose up

.PHONY: all simulator system controllers run