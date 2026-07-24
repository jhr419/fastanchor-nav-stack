# Third-party Packages

This directory contains upstream ROS2 packages used by FastAnchor.

## Packages

- FAST_LIO: LiDAR-Inertial Odometry frontend.
- livox_ros_driver2: Livox LiDAR driver.

These packages are kept as close to upstream as possible. FastAnchor does not modify their internal structure. FastAnchor interacts with them through ROS2 topics, parameters, launch files, and TF.
