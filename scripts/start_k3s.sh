#!/bin/bash
set -e

if [[ $# -ge 1 || "$1" = "--help" ]]
then
    echo "Script downloads and runs k3s on the system"
    echo "Default runs the system with the --docker parameter"
    exit -1
fi


# Only needs to be run once per system
# Download and start kubernetes master node 
echo "Downloading and Running K3s in systemd (Will not do anything if k3s already installed and running"
curl -sfL https://get.k3s.io | INSTALL_K3S_EXEC="--docker" sh -

echo "All actions completed"
echo "k3s will run in the background systemd permanently"
echo "run 'k3s-uninstall.sh' to remove all of k3s"
