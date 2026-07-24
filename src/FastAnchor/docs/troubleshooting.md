# Troubleshooting

## Node Waits For Odometry

Check that FAST-LIO is publishing the configured odometry topic:

```bash
ros2 topic echo /Odometry --once
```

## Node Waits For Initial Pose

Publish `/initialpose` from RViz with the 2D Pose Estimate tool, or set
`use_initial_pose_param: true` and configure `initial_pose_xyz` /
`initial_pose_rpy`.

## ICP Map Is Empty

Verify the configured PCD paths:

- `map_pcd_path`
- `visualization_map_pcd_path`
- `icp_map_pcd_path`

## Third-party Launch Include Fails

`fast_anchor_mid360.launch.py` uses the `lidar_model` launch argument to select:

- `mid360`: package `livox_ros_driver2`, launch `launch_ROS2/msg_MID360_launch.py`
- `mid360s`: package `livox_ros_driver2`, launch `launch_ROS2/msg_MID360s_launch.py`
- package `fast_lio`, launch `launch/mapping.launch.py`

If your upstream packages differ, update the bringup launch file or pass a
custom launch composition without modifying third-party package internals.
