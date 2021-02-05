all: simulator system controllers

simulator:
	$(MAKE) -C simulator

system:
	$(MAKE) -C system

controllers:
	$(MAKE) -C controllers

run:
	docker-compose up

.PHONY: all simulator system controllers run