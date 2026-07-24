# Dependency Guide

FastAnchor expects a ROS2 environment with:

- `rclcpp`
- `sensor_msgs`, `nav_msgs`, `geometry_msgs`, `std_msgs`, `std_srvs`
- `tf2`, `tf2_ros`, `tf2_geometry_msgs`
- PCL, `pcl_ros`, `pcl_conversions`
- `rosidl_default_generators` for `fast_anchor_interfaces`

Third-party workspace packages:

- `fast_lio` from `src/third_party/FAST_LIO`
- `livox_ros_driver2` from `src/third_party/livox_ros_driver2`

Build from the workspace root:

```bash
colcon build --symlink-install
source install/setup.bash
```
