#!/bin/bash

SIMSTART_DELAY=5
OUTPUT_DELAY=5

echo "Waiting for simulation to start"
sleep $SIMSTART_DELAY

echo "Testing connection"
topic_docs=$(PYTHONUNBUFFERED=1 timeout ${OUTPUT_DELAY}s ros2 topic echo /vehicle_1/mavros/state)

echo "${topic_docs}"

# Check if last doc received has connected = True
# NB: -2 subscript as trailing '---' means final doc is seen as <None> by PyYAML
connected=$(echo "${topic_docs}" | python3 -c 'import sys, yaml; y=yaml.safe_load_all(sys.stdin.read()); print(list(y)[-2]["connected"])')

# Result of test command is result of test/container
[[ "${connected}" == "True" ]]
