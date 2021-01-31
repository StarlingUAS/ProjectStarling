all: simulator system

simulator:
	$(MAKE) -C simulator

system:
	$(MAKE) -C system

run:
	docker-compose up

.PHONY: all simulator system run