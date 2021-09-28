#!/bin/bash
set -e

echo "----- Start: $0 -----"
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
        *)    # unknown option
            POSITIONAL+=("$1") # save it in an array for later
            shift # past argument
            ;;
    esac
done
set -- "${POSITIONAL[@]}" # restore positional parameters

if [[ $HELP ]]
then
    echo "Script applies the minimum required for starling"
    echo "(1) Dashboard is started (2) Starling mission UI is started"
    echo "If already started nothing will happen"
    echo "Usage: ./start_starling_base.sh [-h | --help] [-ow | --open]"
    echo "--open will automatically open the associated web pages"
    exit -1
fi

if [[ ! "systemctl is-active --quiet k3s" ]]
then
    echo "k3s is not active as a systemd"
    echo "Please start and download k3s using start_k3s.sh"
    exit -1
fi

SCRIPTSDIR="${BASH_SOURCE%/*}"
SYSTEMDIR="$SCRIPTSDIR/../system"
DEPLOYMENTDIR="$SCRIPTSDIR/../deployment"

# Start dashboard
./$SCRIPTSDIR/start_dashboard.sh

# Start starling UI
kubectl apply -f "$DEPLOYMENTDIR/k8.starling-ui-dashly.yaml"

if [[ $OPENWEBPAGE ]]
then
    echo "Opening Starling UI to http://localhost:3000"
    xdg-open http://localhost:3000

    echo "Opening Dashboard to https://localhost:31771"
    xdg-open https://localhost:31771
fi
echo "----- End: $0 -----"
