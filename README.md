# Running the simulator image

At the moment, usinga graphical simulator and passing through Xserver from host.
For OpenGL stuff need to have nvidia-container-toolkit installed on the host:
https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html

In summary add nvidia docker repos:
```sh
distribution=$(. /etc/os-release;echo $ID$VERSION_ID) \
   && curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add - \
   && curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list
```

Install
```sh
sudo apt update
sudo apt install nvidia-docker2
sudo systemctl daemon-reload
sudo systemctl restart docker
```