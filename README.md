# FastAnchor + FastPlanner + SCAN-Planner

This workspace contains a minimal ROS 2 connection from FastAnchor localization,
through the FastPlanner A* global planner, to SCAN-Planner and the chassis command.
The copied upstream modules remain separate; no navigation adapter is required because
their existing ROS 2 message types match.

## Build

```bash
source /opt/ros/humble/setup.bash
cd <workspace>
colcon build --symlink-install
source install/setup.bash
```

## Start

Pass the same PCD map to FastAnchor and FastPlanner:

```bash
ros2 launch navigation_bringup navigation_system.launch.py \
  map_pcd_path:=<absolute-path-to-map.pcd>
```

The default frames are `map -> odom -> base_link`. The launch arguments
`localization_pose_topic`, `goal_topic`, `global_path_topic`, and `cmd_vel_topic`
only change interface names and do not alter planner behavior.

## Data flow

| Module | Input | Output | ROS 2 type |
|---|---|---|---|
| FastAnchor | `/initialpose` | `/fast_anchor/odom` | `geometry_msgs/msg/PoseWithCovarianceStamped` -> `nav_msgs/msg/Odometry` |
| FastPlanner A* | `/fast_anchor/odom`, `/move_base_simple/goal` | `/planned_path` | `nav_msgs/msg/Odometry`, `geometry_msgs/msg/PoseStamped` -> `nav_msgs/msg/Path` |
| SCAN-Planner | `/fast_anchor/odom`, `/fast_anchor/aligned_cloud`, `/planned_path` | `/planning/bspline` | `nav_msgs/msg/Odometry`, `sensor_msgs/msg/PointCloud2`, `nav_msgs/msg/Path` -> `scan_planner_msgs/msg/Bspline` |
| SCAN controller | `/planning/bspline`, `/fast_anchor/odom` | `/cmd_vel` | `scan_planner_msgs/msg/Bspline`, `nav_msgs/msg/Odometry` -> `geometry_msgs/msg/Twist` |

The existing blue/green interactive markers are reused. The initial-pose marker publishes
to FastAnchor. In integrated mode the goal marker publishes only to FastPlanner; the direct
single-waypoint path publication is disabled so SCAN-Planner only receives FastPlanner output.
