all: simulator system controllers

simulator:
	$(MAKE) -C simulator

system:
	$(MAKE) -C system

controllers:
	$(MAKE) -C controllers

docs:
	mkdocs build

serve-docs:
	mkdocs serve -a 0.0.0.0:8000

run:
	docker-compose up

.PHONY: all simulator system controllers run