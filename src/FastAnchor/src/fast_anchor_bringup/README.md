# fast_anchor_bringup

Launch, configuration, and RViz entry points for FastAnchor.

## Common Commands

```bash
ros2 launch fast_anchor_bringup localization_only.launch.py
ros2 launch fast_anchor_bringup fast_anchor_mid360.launch.py
ros2 launch fast_anchor_bringup fast_anchor_mid360.launch.py lidar_model:=mid360
ros2 launch fast_anchor_bringup bag_localization.launch.py bag_path:=/path/to/bag
ros2 launch fast_anchor_bringup rviz.launch.py
```

`lidar_model` accepts `mid360s` (default) and `mid360`. The selected model loads
`msg_MID360_launch.py` or `msg_MID360s_launch.py` from `livox_ros_driver2`.
`fast_anchor_mid360.launch.py` also includes `fast_lio`. If a third-party launch
file cannot be included, verify the installed package and launch file names.
