#!/bin/bash
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh

# Prepare for installing ros2
sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt upgrade

# Install a metapackage with ros2
sudo apt install ./robotics-metapackage_0.1_all.deb


# MoveIt installation
# Taken from https://moveit.picknik.ai/humble/doc/tutorials/getting_started/getting_started.html

# Initialize and update rosdep, the ROS system dependency manager
sudo rosdep init
rosdep update

# Install Colcon the ROS 2 build system, with mixin:
sudo apt install python3-colcon-common-extensions
sudo apt install python3-colcon-mixin
colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
colcon mixin update default

# Install vcstool
sudo apt install python3-vcstool

# Check if src/moveit2_tutorials directory exists
# If not, download source code of moveit and tutorials
if [ ! -d "src/moveit2_tutorials" ]; then
    cd src
    git clone https://github.com/ros-planning/moveit2_tutorials -b humble --depth 1
    vcs import < moveit2_tutorials/moveit2_tutorials.repos
    cd -
fi

# Install dependencies of MoveIt through rosdep
source /opt/ros/humble/setup.bash
cd src
sudo apt update && rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
colcon build --mixin release