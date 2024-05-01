#!/bin/bash

UBUNTU_VERSION=$(lsb_release -rs)
if [ "$UBUNTU_VERSION" == "22.04" ]; then
    ROS2_DEFAULT_VERSION="humble"
elif [ "$UBUNTU_VERSION" == "20.04" ]; then
    ROS2_DEFAULT_VERSION="foxy"
elif [ "$UBUNTU_VERSION" == "18.04" ]; then
    ROS2_DEFAULT_VERSION="dashing"
else
    echo "Unsupported Ubuntu version: $UBUNTU_VERSION"
    exit 1
fi

ROS2_VERSION=${ROS2_VERSION:-$ROS2_DEFAULT_VERSION}


# Activate ROS2 environment
source "/opt/ros/$ROS2_VERSION/setup.bash"


vcs import src < robotics.repos

rosdep install --from-paths src -i --rosdistro "$ROS2_VERSION"
colcon build --symlink-install --executor sequential
echo "Colcon build success! To activate your workspace: source install/setup.bash"
