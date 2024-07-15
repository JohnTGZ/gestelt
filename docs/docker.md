# Docker

# Official ROS docker images
They can be found on the [docker hub registry](https://registry.hub.docker.com/_/ros/). We are currently using `noetic-ros-core-focal`.

# Installing docker
```bash
curl -fsSL test.docker.com -o get-docker.sh && sh get-docker.sh
sudo usermod -aG docker $USER 

# Test the installation
docker run hello-world 

# [TROUBLESHOOTING] If there is an error with the repository not being signed, add this:
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 9B98116C9AA302C7
```


# Host

## Building and Pushing images to Docker Hub repository
```bash
docker build -t johntgz95/radxa-gestelt:latest .

# Tag image
docker tag gestelt/ros:noetic johntgz95/gestelt
docker push johntgz95/radxa-gestelt:latest

# All-in-one build and push
docker build --platform linux/arm64 -t johntgz95/radxa-base:latest --push .
docker build --platform linux/arm64 -t johntgz95/radxa-gestelt:latest --push .
```

## Running containers
```bash
# To use host USB devices, add "--privileged" flag or "--device=/dev/ttyAML1"
docker run -it --rm --network host --privileged johntgz95/radxa-gestelt:latest
docker run -it --network host --privileged johntgz95/radxa-gestelt:latest

# Find name of new machine 
docker ps -l

# Start additional bash sessions in same container
docker exec -it <container_id> bash

# List all docker containers
docker ps -a
# Start stopped container
docker start <container_id>
docker exec -it <container_id> bash
# Stop all containers
docker stop $(docker ps -a -q)
# Remove all containers
docker rm $(docker ps -a -q)
```

## Make changes to containers and save them as new images
```bash
# List all docker containers
docker ps -a
# Commit changes
docker commit <CONTAINER_ID> johntgz95/radxa-gestelt:latest
# push 
docker push johntgz95/radxa-gestelt:latest
```

# Radxa

## Commands
```bash
# Pull Images
docker pull johntgz95/radxa-gestelt:latest
docker run -it --rm --privileged johntgz95/radxa-gestelt:latest
```

# Repositories
[Radxa-base](https://hub.docker.com/repository/docker/johntgz95/radxa-base/general)

# Resources
[Uploading an image to a Docker Hub repo](https://docs.docker.com/guides/workshop/04_sharing_app/).

[A Guide to Docker and ROS](https://roboticseabass.com/2021/04/21/docker-and-ros/)