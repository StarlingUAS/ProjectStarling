#!/bin/bash
set -e

# Download and start kubernetes master node 
echo "Downloading and Running K3s in systemd"
curl -sfL https://get.k3s.io | INSTALL_K3S_EXEC="--docker" sh -

echo "Deploying Gazebo-iris to 10.43.226.5:8080 and wait for start"
sudo k3s kubectl apply -f deployment/k8.starling-gazebo.amd64.yaml

sleep 5s

echo "Deploying px4-sitl with mavros"
sudo k3s kubectl apply -f deployment/k8.px4-sitl.amd64.yaml

echo "Deploying web ui"
sudo k3s kubectl apply -f system/ui/kubernetes.yaml

echo "Deploying Dashboard"
echo "==================="
GITHUB_URL=https://github.com/kubernetes/dashboard/releases
VERSION_KUBE_DASHBOARD=$(curl -w '%{url_effective}' -I -L -s -S ${GITHUB_URL}/latest -o /dev/null | sed -e 's|.*/||')
sudo k3s kubectl apply -f https://raw.githubusercontent.com/kubernetes/dashboard/${VERSION_KUBE_DASHBOARD}/aio/deploy/recommended.yaml
sudo k3s kubectl apply -f deployment/resources/dashboard.admin-user.yml -f deployment/resources/dashboard.admin-user-role.yml
sudo k3s kubectl proxy 2>/dev/null & 
echo "==================="

echo "Here is the dashboard token, use it to log in to the dashboard."
sudo k3s kubectl -n kubernetes-dashboard describe secret admin-user-token | grep ^token

echo "Opening Starling UI"
xdg-open http://localhost:30000

echo "Opening gazebo"
xdg-open http://10.43.226.5:8080

echo "Opening Dashboard"
xdg-open http://localhost:8001/api/v1/namespaces/kubernetes-dashboard/services/https:kubernetes-dashboard:/proxy/


