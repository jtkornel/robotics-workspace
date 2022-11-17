#!/bin/bash

# Running ROS2 in docker requires some specific settings for network transparency
# https://answers.ros.org/question/370595/ros2-foxy-nodes-cant-communicate-through-docker-container-border/
# NOTE: this is only known to work on x86_64

docker run --name ros2_interactive --network host --pid host -v /dev/shm:/dev/shm --rm -it -v $(pwd):${pwd}/robotics-workspace ros_docker:latest
