#!/bin/bash
set -e
ARGS="$@"
# Parse arguments
POSITIONAL=()
while [[ $# -gt 0 ]]
do
    key="$1"
    case $key in
        -h|--help)
            HELP=1
            shift # past argument
            ;;
        -d|--delete)
            DELETE=1
            shift
            ;;
        *)    # unknown option
            POSITIONAL+=("$1") # save it in an array for later
            shift # past argument
            ;;
    esac
done
set -- "${POSITIONAL[@]}" # restore positional parameters

if [[ $HELP ]]
then
    echo "Script installs and runs k3s, then starts gazebo and a single px4 sitl drone"
    echo "It runs start_k3s_agent.sh and start_single_px4sitl_gazebo.sh from scripts dir"
    echo "Usage: './run_k3s.sh [-h|--help] [-d|--delete]"
    echo "-h|--help: prints this dialogue"
    echo "-d|--delete: removes everything k3s on the system (k3s-uninstall)"
    exit -1
fi

SCRIPTSDIR=scripts

# Start K3S
./$SCRIPTSDIR/start_k3s_agent.sh $ARGS

if [ ! $DELETE ]; then
    # Start Gazebo and Single PX4 drone
    ./SCRIPTSDIR/start_single_px4sitl_gazebo.sh $ARGS
fi
