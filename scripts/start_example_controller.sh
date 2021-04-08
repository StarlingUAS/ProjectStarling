#!/bin/bash
set -e


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
        -r|--restart)
            RESTART=1
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
    echo "This script builds and runs the example controller"
    echo "Usage: ./start_example_controller.sh [-h|--help] [-d|--delete] [-r|--restart]"
    exit -1
fi


SCRIPTSDIR="${BASH_SOURCE%/*}"
CONTROLLERSDIR="$SCRIPTSDIR/../controllers"

if [[ $DELETE ]]; then
    kubectl delete -f $CONTROLLERSDIR/example_controller_python/k8.example_controller_python.amd64.yaml
fi

if [[ $RESTART ]]; then
    echo "Restarting, waiting 20 seconds for clean shutdown"
    sleep 20
fi

if [[ ! $DELETE || $RESTART ]]; then
    kubectl apply -f $CONTROLLERSDIR/example_controller_python/k8.example_controller_python.amd64.yaml
fi