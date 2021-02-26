#!/bin/bash
set -e

# Download and start kubernetes master node 
echo "Downloading and Running K3s in systemd (Will not do anything if k3s already installed and running"
curl -sfL https://get.k3s.io | INSTALL_K3S_EXEC="--docker" sh -

echo "Deploying Kubernetes Dashboard"
echo "See: https://blog.tekspace.io/deploying-kubernetes-dashboard-in-k3s-cluster/ for more details"
echo "==================="
GITHUB_URL=https://github.com/kubernetes/dashboard/releases
VERSION_KUBE_DASHBOARD=$(curl -w '%{url_effective}' -I -L -s -S ${GITHUB_URL}/latest -o /dev/null | sed -e 's|.*/||')
sudo k3s kubectl apply -f https://raw.githubusercontent.com/kubernetes/dashboard/${VERSION_KUBE_DASHBOARD}/aio/deploy/recommended.yaml
sudo k3s kubectl apply -f deployment/resources/dashboard.admin.yaml #dashboard.admin-user.yml -f deployment/resources/dashboard.admin-user-role.yml
echo "==================="

echo "Here is the dashboard token, use it to log in to the dashboard."
DASHBOARD_TOKEN=`sudo k3s kubectl -n kubernetes-dashboard describe secret admin-user-token | grep ^token | cut -c 13-`
echo $DASHBOARD_TOKEN
echo $DASHBOARD_TOKEN | xclip -selection clipboard -i
echo "The token has been copied onto your clipboard"
echo "Note: your browser may not like the self signed ssl certificate, ignore and containue for now"
echo "To get the token yourself run: sudo k3s kubectl -n kubernetes-dashboard describe secret admin-user-token"
echo "==================="

echo "Deploying Starling Modules (this may take a while)"
echo "Deploying Gazebo-iris to localhost:8080 and wait for start"
sudo k3s kubectl apply -f deployment/k8.gazebo-iris.amd64.yaml

echo "Deploying web ui"
sudo k3s kubectl apply -f system/ui/kubernetes.yaml

echo "Waiting 20s for Gazebo to start to ensure proper connection"
sleep 20s

echo "Deploying px4-sitl with mavros"
sudo k3s kubectl apply -f deployment/k8.px4-sitl.amd64.yaml

echo "Wait for containers to initialise"
sleep 10s

echo "Opening Starling UI to http://localhost:3000/html/main.html"
xdg-open http://localhost:3000/html/main.html

echo "Opening gazebo to http://localhost:8080"
xdg-open http://localhost:8080

echo "Opening Dashboard to https://localhost:31771"
xdg-open https://localhost:31771

echo "All actions completed"
echo "k3s will run in the background systemd permanently"
echo "run 'k3s-uninstall.sh' to remove all of k3s"
