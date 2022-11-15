#!/bin/bash
ROS2_VERSION=${ROS2_VERSION:-foxy}

# Install docker if necessary
if ! [ -x "$(command -v docker)" ]; then
    curl -fsSL https://get.docker.com -o get-docker.sh
fi

# Docker images for ROS/ROS2
if [ ! -d "src/docker_images" ]; then
    git clone git@github.com:osrf/docker_images.git
fi

# Install newuidmap & newgidmap binaries
sudo apt-get install -y uidmap

dockerd-rootless-setuptool.sh install

cd docker_images/ros/$ROS2_VERSION/ubuntu/focal/desktop
docker build -t ros_docker .
