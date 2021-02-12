
K3S_TOKEN=`sudo cat /var/lib/rancher/k3s/server/node-token`
K3S_SERVER=https://192.168.0.63:6443
INSTALL_K3S_EXEC="--docker"

echo "Starting specified k8 agents"
echo "K3S Token is $K3S_TOKEN"

run_k3s_agent () {
    echo "Connecting to $2 with user $1, starting k3s agent node named $3"
    ssh $1@$2 "curl -sfL https://get.k3s.io | K3S_NODE_NAME=$3 K3S_URL=${K3S_SERVER} K3S_TOKEN=${K3S_TOKEN} INSTALL_K3S_EXEC=${INSTALL_K3S_EXEC} sh -;";
}

run_k3s_agent ubuntu 192.168.0.110 clover1
run_k3s_agent ubuntu 192.168.0.96 raspi1
