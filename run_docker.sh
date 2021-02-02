#!/usr/bin/env bash

swarm_state=$(docker info 2> /dev/null | grep Swarm | awk '{print $2}')

[ ! $swarm_state == active ] && docker swarm init

docker stack deploy -c docker-compose.swarm.yml multisim