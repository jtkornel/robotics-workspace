#!/bin/bash

UBUNTU_VERSION=$(lsb_release -rs)
if [ $UBUNTU_VERSION == "22.04" ]; then
    ROS2_DEFAULT_VERSION="humble"
elif [ $UBUNTU_VERSION == "20.04" ]; then
    ROS2_DEFAULT_VERSION="foxy"
elif [ $UBUNTU_VERSION == "18.04" ]; then
    ROS2_DEFAULT_VERSION="dashing"
else
    echo "Unsupported Ubuntu version: $UBUNTU_VERSION"
    exit 1
fi

ROS2_VERSION=${ROS2_VERSION:-$ROS2_DEFAULT_VERSION}

sudo apt-get install curl
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh

# Prepare for installing ros2 packages
sudo apt install software-properties-common
sudo add-apt-repository universe -y

sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo curl -sSL https://apt.kitware.com/kitware-archive.sh
sudo bash kitware-archive.sh 

sudo apt update
sudo apt upgrade

sudo apt-get install cmake
sudo apt-get install ros-$ROS2_VERSION-slam-toolbox

# Initialize and update rosdep, the ROS system dependency manager
sudo rosdep init
rosdep update

# Install Colcon the ROS 2 build system
sudo apt install python3-colcon-common-extensions

# Install vcstool
sudo apt install python3-vcstool

# Activate ROS2 environment
source /opt/ros/$ROS2_VERSION/setup.bash


vcs import src < robotics.repos

rosdep install --from-paths src -i --rosdistro $ROS2_VERSION
colcon build --symlink-install --executor sequential
echo "Colcon build success! To activate your workspace: source install/setup.bash"
