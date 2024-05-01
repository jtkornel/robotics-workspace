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

echo "Installing ROS2 $ROS2_VERSION"

# Prepare for installing ros2
sudo apt install software-properties-common
sudo add-apt-repository universe -y

sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo "$UBUNTU_CODENAME") main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt upgrade

sudo apt install "ros-$ROS2_VERSION-desktop" ros-dev-tools python3-rosdep python3-colcon-common-extensions -y
sudo apt install "ros-$ROS2_VERSION-navigation2" "ros-$ROS2_VERSION-nav2-bringup"
