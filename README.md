# robotics-workspace
Template to bootstrap a ROS2 &amp; create_robot workspace with docker or native Ubuntu (22.04)

Either run `bash install_ros2_docker.sh` to install ROS2 docker images (including docker if necessary)

Or run `bash install_ros2_native.sh` to install ROS2 natively (from the robotics-metapackage deb)


From the the environment with ROS2 installed, run `bash workspace_setup.sh` to build a ROS2 workspace with create_robot driver for iRobot Create 1/2.


NOTE: If the colcon build fails with old references in the ROS2 workspace setup you may have to clean up the cmake build cache
`colcon build --cmake-clean-cache`


See create_robot instructions here
https://github.com/AutonomyLab/create_robot/tree/foxy

