#!/bin/bash

if [[ $# -gt 1 ]]; then
	COMPOSE_FILE=$1
	shift
	docker-compose -f $COMPOSE_FILE "$@"
else
	docker-compose -f $1 up --exit-code-from test
fi

