#!/bin/bash
set -e

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

SCRIPTSDIR="${BASH_SOURCE%/*}"
SYSTEMDIR="$SCRIPTSDIR/../system"
DEPLOYMENTDIR="$SCRIPTSDIR../deployment"

# Start dashboard
./$SCRIPTSDIR/start_dashboard.sh

# Start starling UI
sudo k3s kubectl apply -f "$SYSTEMDIR/ui/kubernetes.yaml"

if [[ $OPENWEBPAGE ]]
then
    echo "Opening Starling UI to http://localhost:3000/html/main.html"
    xdg-open http://localhost:3000/html/main.html

    echo "Opening Dashboard to https://localhost:31771"
    xdg-open https://localhost:31771
fi

