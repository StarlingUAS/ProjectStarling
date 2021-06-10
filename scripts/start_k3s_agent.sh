#!/bin/bash

if [[ $# -le 1 || $# -ge 4 || "$1" = "help" ]]
then
    echo "Script to start k3s-agent on another machine"
    echo "It ssh's into a remote machine and runs 'k3s-install.sh' with this machines Token"
    echo "Usage: ./run_k3s_agent.sh <Remote (Root enabled) Username> <Remote IP Address> <K3S Node Name>"
    echo "e.g ./start_k3s_agent ubuntu 192.168.0.110 clover1"
    echo "e.g. K3S_SERVER=https://192.168.0.63:6443 k3s_agent ubuntu 192.168.0.96 raspi1"
    echo ""
    echo "Note: It is assumed that the k3s-airgap-images-arm64.tar and the k3s-arm64 files have been downloaded from the k3s github releases and are in the home directory named as such."
    exit -1
fi

echo "Starting specified k3a agents"

K3S_TOKEN=`sudo cat /var/lib/rancher/k3s/server/node-token`
K3S_SERVER="${K3S_SERVER:-https://192.168.0.63:6443}"
INSTALL_K3S_EXEC="--docker"

echo "K3S Token is $K3S_TOKEN"
echo "K3S Server is $K3S_SERVER"
echo "Connecting to $2 with user $1, starting k3s agent node named $3"

ssh $1@$2 " \
 k3s-agent-uninstall.sh;\
 sudo mkdir -p /var/lib/rancher/k3s/agent/images/; \
 sudo cp k3s-airgap-images-arm64.tar /var/lib/rancher/k3s/agent/images/; \ 
 sudo chmod +x k3s-arm64; \
 sudo cp k3s-arm64 /usr/local/bin/k3s; \
 INSTALL_K3S_SKIP_DOWNLOAD=true K3S_NODE_NAME=$3 K3S_URL=${K3S_SERVER} K3S_TOKEN=${K3S_TOKEN} INSTALL_K3S_EXEC=${INSTALL_K3S_EXEC} ./k3s_install.sh;";


exit -1