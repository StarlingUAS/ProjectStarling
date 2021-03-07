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
        *)    # unknown option
            POSITIONAL+=("$1") # save it in an array for later
            shift # past argument
            ;;
    esac
done
set -- "${POSITIONAL[@]}" # restore positional parameters

if [[ $HELP ]]
then
    echo "Script starts the kubernetes dashboard and reports the connection token"
    echo "Assumes that kubernetes is already running on the system"
    exit -1
fi

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