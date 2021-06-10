# Using `kubectl` from Remote Machines

To use `kubectl` from a remote machine, a copy of the `k3s.yaml` from the master machine is needed.

On the machine you want to use `kubectl` from, run:

`export KUBECONFIG=/path/to/k3s.yaml`

This adds the `KUBECONFIG` variable to the environment. To make it permanent, it needs to be added to the `.bashrc`.

Once this is done, `kubectl` can be used as normal and will connect to the master. To test, run:

`kubectl cluster-info`

You should see `Kubernetes control plane is running at...` reflecting your Kubernetes master.
