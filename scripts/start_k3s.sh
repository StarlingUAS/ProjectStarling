#!/bin/bash
set -e

KUBECONFIGDIR=~/.kube
CONFIGFILE=$KUBECONFIGDIR/config/k3s.yaml

declare -a RCFILES=(
    ~/.zshrc
    ~/.bashrc
)

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
        -u|--uninstall)
            UNINSTALL=1
            shift
            ;;
        --config-file)
            CONFIGFILE=$2 
            shift
            shift
            ;;
        --do-not-start)
            SKIPENABLE=1
            shift
            ;;
        --node-external-ip)
            EXTERNAL_IP=$2
            shift
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
    echo "Script downloads and runs k3s on the system"
    echo "Default runs the system with the --docker parameter"
    echo "k3s config file can be set with --config-file <filepath>, defaults to $CONFIGFILE"
    exit -1
fi

if [[ ! $UNINSTALL ]]; then

    echo "Created directory $(dirname "${CONFIGFILE}") for storing config file"
    mkdir -p "$(dirname "${CONFIGFILE}")"

    K3S_ADDITIONAL_ARGS="--docker --write-kubeconfig $CONFIGFILE --write-kubeconfig-mode  644"

    if [[ $EXTERNAL_IP ]]; then
        echo "External IP set to ${EXTERNAL_IP}"
        K3S_ADDITIONAL_ARGS="${K3S_ADDITIONAL_ARGS} --node-external-ip ${EXTERNAL_IP}"
    fi

    # Only needs to be run once per system
    # Download and start kubernetes master node 
    echo "Downloading and Running K3s in systemd (Will not do anything if k3s already installed and running"
    echo "The configuration file will be placed in $CONFIGFILE"
    echo "root is required for initial installation (running of the kubernetes systemd)"
    curl -sfL https://get.k3s.io | sudo bash -s - ${K3S_ADDITIONAL_ARGS}

    echo "Setting KUBECONFIG in rcfiles"
    for rcfile in "${RCFILES[@]}"; do
        if [[ -f $rcfile ]]; then
            case `grep -F KUBECONFIG $rcfile >/dev/null; echo $?` in
                0)
                    echo "KUBECONFIG already set in $rcfile, KUBECONFIG NOT added to $rcfile"
                    ;;
                1)
                    echo "export KUBECONFIG=$CONFIGFILE" >> $rcfile
                    echo "$rcfile appended KUBECONFIG, set to $CONFIGFILE"
                    ;;
                *)
                    echo "Error occured in setting KUBECONFIG in $rcfile"
                    RED='\033[0;31m'
                    NC='\033[0m'
                    echo "${RED}Please manually ensure that KUBECONFIG is set to $CONFIGFILE in .profile or .bashrc file${NC}"
                    ;;
            esac
        else
            echo "$rcfile not found"
        fi
    done
    
    echo "All actions completed"
    echo "k3s will run in the background systemd permanently"
    echo "run this script with the --uninstall flag or 'k3s-uninstall.sh' to remove all of k3s"
else
    echo "Removing k3s"
    sudo k3s-uninstall.sh
    echo "Removed k3s from system"
    sudo rm -r $KUBECONFIGDIR
    echo "Removed k3s config from $KUBECONFIGDIR"
    exit 1
fi

if [[ $SKIPENABLE ]]; then
    k3s-killall.sh
fi
