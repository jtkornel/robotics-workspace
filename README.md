# robotics-workspace
Template to bootstrap a ROS2 &amp; create_robot workspace on Ubuntu 22.04

Run `bash workspace_setup.sh` to download and install ROS 2 Humble and build a workspace with create_robot driver for iRobot Create 1/2.

If the build fails with old references you may have to clean up the build cache
`colcon build --cmake-clean-cache`

To activate your workspace and start using it you must source the generated setup file:
`source ./install/setup.bash`

See create_robot instructions here
https://github.com/AutonomyLab/create_robot/tree/foxy

