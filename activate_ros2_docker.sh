#!/bin/bash
docker run --name ros2_interactive --rm -it -v $(pwd):${pwd}/robotics-workspace ros_docker:latest
