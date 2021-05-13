#!/bin/bash
set -e

echo "----- Start: $0 -----"

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
        -ow|--open)
            OPENWEBPAGE=1
            shift # past argument
            ;;
        -d|--delete)
            DELETE=1
            shift
            ;;
        -sk|--skip-base-check)
            SKIP_BASE=1
            shift
            ;;
        -r|--restart)
            DELETE=1
            RESTART=1
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
    echo "Script starts both gazebo and a single px4 instance"
    echo "If the base is not running, this script will start the base"
    echo "If already started nothing will happen"
    echo "Usage: ./start_single_px4sitl_gazebo.sh [-h | --help] [-ow | --open]"
    echo "--open will automatically open the associated web pages"
    exit -1
fi

SCRIPTSDIR="${BASH_SOURCE%/*}"
SYSTEMDIR="$SCRIPTSDIR/../system"
DEPLOYMENTDIR="$SCRIPTSDIR/../deployment"


if [[ $DELETE ]]; then
    echo "Deleting Gazebo-iris"
    kubectl delete -f $DEPLOYMENTDIR/k8.gazebo-iris.amd64.yaml
    echo "Deleting px4-sitl with mavros"
     kubectl delete -f $DEPLOYMENTDIR/k8.px4-sitl.amd64.yaml
fi

if [[ $RESTART ]]; then
    echo "Restarting, sleeping 30";
    sleep 30;
fi

if [[ ! $DELETE || $RESTART ]]; then

    if [[ ! $SKIP_BASE ]]; then
        # Start/ Check starling base
        echo "Starting Starling Base, args: $ARGS"
        ./$SCRIPTSDIR/start_starling_base.sh $ARGS
    else
        echo "Skipping Base Check"
    fi

    echo "Deploying Starling Modules (this may take a while)"
    echo "Deploying Gazebo-iris to localhost:8080 and wait for start"
    kubectl apply -f $DEPLOYMENTDIR/k8.gazebo-iris.amd64.yaml

    echo "Waiting 5s for Gazebo to start to ensure proper connection"
    sleep 5s

    echo "Deploying px4-sitl with mavros"
    kubectl apply -f $DEPLOYMENTDIR/k8.px4-sitl.amd64.yaml

    if [[ $OPENWEBPAGE ]]
    then
        echo "Opening gazebo to http://localhost:8080"
        xdg-open http://localhost:8080
    fi
fi

echo "----- End: $0 -----"