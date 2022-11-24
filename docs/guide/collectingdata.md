# Collecting Data

[TOC]

## Inside and Outside of a Container

When working with containers, you need to be careful to whether you are working inside or ourside of it. 
All applications within a Docker container will be run inside the file system of the Docker container. 
Therefore any files saved or changed will always be temporary for the lifetime of the container. 

This is contrasted with outside of your container, i.e. your local file system. The container does not know about the outside world and your file system unless you tell it. 

In order to save files to the outside of a container, you can make use of either Mounting or Volumes. 
This allows you to map a folder on your local file system to a folder inside the docker container. 

For example if I wanted to save data in `/home/myuser/Documents/data`. I could mount this folder into the filesystem as `/data`. Then in your application, you can save your files inside `/data`, and these in turn will be saved inside `/home/myuser/Documents/data`.

## Mounting a file or folder with Docker run

When you use Docker run use the `--volume` or `-v` option:
```
docker run ... -v "$(pwd)"/home/myuser/Documents/data:/data uobflightlabstarling/my-container
```

See [here](https://docs.docker.com/storage/bind-mounts/) for more information

## Mounting using Docker compose 

In the docker compose file you can specify "volumes" 

```
volumes:
  # Just specify a path and let the Engine create a volume
  - /var/lib/mysql

  # Specify an absolute path mapping
  - /opt/data:/var/lib/mysql
```

Using the same syntax as above.

See [here](https://docs.docker.com/compose/compose-file/compose-file-v3/#volumes) for more information

## Mounting in Kubernetes

You have to specify a volume first, and then mount that volume into the container. See [here](https://www.deploycontainers.com/2022/02/03/mount-local-volume-with-kubernetes/) or have a look at some of the deployment files in Murmuration. 

