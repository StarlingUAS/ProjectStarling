#!/bin/bash

set -e
echo "Start of script"

echo "Startling discovery service"
# Sleep pipe required to capture stray input which stops the discovery server
sleep infinity | fastdds discovery -i 0 -l 0.0.0.0 -p 11811 
echo "Finished discovery service"
