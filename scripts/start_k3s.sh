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
        -u|--uninstall)
            UNINSTALL=1
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
    exit -1
fi

if [[ ! $UNINSTALL ]]; then
    # Only needs to be run once per system
    # Download and start kubernetes master node 
    echo "Downloading and Running K3s in systemd (Will not do anything if k3s already installed and running"
    curl -sfL https://get.k3s.io | INSTALL_K3S_EXEC="--docker" sh -

    echo "All actions completed"
    echo "k3s will run in the background systemd permanently"
    echo "run this script with the --uninstall flag or 'k3s-uninstall.sh' to remove all of k3s"
else
    echo "Removing k3s"
    k3s-uninstall.sh
fi