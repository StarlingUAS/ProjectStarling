#!/bin/bash
set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

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
echo "See: https://rancher.com/docs/k3s/latest/en/installation/kube-dashboard/ for more details"
echo "==================="
GITHUB_URL=https://github.com/kubernetes/dashboard/releases
VERSION_KUBE_DASHBOARD=$(curl -w '%{url_effective}' -I -L -s -S ${GITHUB_URL}/latest -o /dev/null | sed -e 's|.*/||')
kubectl apply -f https://raw.githubusercontent.com/kubernetes/dashboard/${VERSION_KUBE_DASHBOARD}/aio/deploy/recommended.yaml
kubectl apply -f ${SCRIPT_DIR}/../deployment/resources/dashboard.admin.yaml #dashboard.admin-user.yml -f deployment/resources/dashboard.admin-user-role.yml
echo "==================="

DASHBOARD_TOKEN=`k3s kubectl -n kubernetes-dashboard describe secret admin-user-token | grep ^token | cut -c 13-`

echo "The Dashboard is available at https://localhost:31771"
echo "You will need the dashboard token, to access it."
if command -v xclip; then
    set +e # Will fail if there is no xserver running, e.g. via ssh. Fail silently
    echo $DASHBOARD_TOKEN | xclip -selection clipboard -i
    echo "The token has been copied onto your clipboard, it is also printed below"
    set -e
else
    echo "Copy and paste the token from below"
fi
echo "-----BEGIN DASHBOARD TOKEN-----"
echo $DASHBOARD_TOKEN
echo "-----END DASHBOARD TOKEN-----"
echo "Note: your browser may not like the self signed ssl certificate, ignore and continue for now"
echo "To get the token yourself run: k3s kubectl -n kubernetes-dashboard describe secret admin-user-token"
echo "==================="
