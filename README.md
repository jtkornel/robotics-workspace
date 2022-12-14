# robotics-workspace
Template to bootstrap a ROS2 &amp; create_robot workspace, with docker or native Ubuntu

## ROS2 install

Choose between native install (recommended ubuntu 20.04/22.04) or docker install (most linuxes supporting docker)

### docker
Run `bash install_ros2_docker.sh` to install ROS2 docker images (includes installing docker if necessary)

To enter the docker container run `bash activate_ros2_docker.sh` - which will mount this workspace on under `/robotics-workspace`

### native
Run `bash install_ros2_native.sh` to install ROS2 natively (from the robotics-metapackage deb)

## Workspace with drivers

From the the environment with ROS2 installed, run `bash workspace_setup.sh` to build a ROS2 workspace with create_robot driver for iRobot Create 1/2.

Remember to activate the ROS2 workspace to make the driver available:
`source install/setup.bash`

For further details on launching create_robot see here:
https://github.com/AutonomyLab/create_robot/tree/foxy

## Troubleshooting

If the colcon build fails with old references in the ROS2 workspace setup you may have to clean up the cmake build cache and/or force rebuild:
`colcon build --cmake-clean-cache --cmake-clean-first`

Maybe also delete _install_ and _build_ folders, which has build products. As last resort delete _src_ and start over
