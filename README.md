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
  map_pcd_path:=$PWD/maps/map_preprocessed2.pcd
```

The integrated launch is headless by default on this performance branch: all
three RViz processes are disabled so onboard visualization cannot consume the
navigation CPU budget. Enable only the views needed for debugging with
`start_localization_rviz:=true`, `start_local_planner_rviz:=true`, or
`start_global_planner_rviz:=true`.

SCAN-Planner selects its local target 4.0 m ahead of the robot by default, inside
the 5.0 m local sensing range. Override it when needed with
`local_target_distance:=<metres>`.

The integrated launch defaults to the connected `mid360s` LiDAR. Use
`lidar_model:=mid360` only when running with the original MID360 model.

Select the LiDAR model at launch time without changing source files:

```bash
# MID360s (default)
ros2 launch navigation_bringup navigation_system.launch.py \
  map_pcd_path:=$PWD/maps/map_preprocessed.pcd lidar_model:=mid360s

# MID360
ros2 launch navigation_bringup navigation_system.launch.py \
  map_pcd_path:=$PWD/maps/map_preprocessed.pcd lidar_model:=mid360
```

The default frames are `map -> odom -> base_link`. The launch arguments
`localization_pose_topic`, `goal_topic`, `global_path_topic`, and `cmd_vel_topic`
only change interface names and do not alter planner behavior.

## CPU-optimized runtime

The optimized defaults preserve 5 Hz ICP, 20 Hz occupancy fusion, and 100 Hz
control. They reduce non-critical work as follows:

- FastAnchor preprocesses every fresh LiDAR cloud and republishes the latest
  aligned result at a fixed 25 Hz from a separate executor thread. ICP itself
  remains limited to 5 Hz, and the full path remains limited to 2 Hz.
- SCAN publishes occupancy visualization at 5 Hz. Its two visualization layers
  share one voxel traversal and serialization runs on a dedicated worker thread,
  allowing Linux to schedule visualization and planning on different CPU cores.
- RViz is opt-in. On a fully headless platform, pass
  `grid_visualization_rate_hz:=0` to remove the remaining grid visualization work.

Restore the previous diagnostic rates without reverting code:

```bash
ros2 launch navigation_bringup navigation_system.launch.py \
  map_pcd_path:=$PWD/maps/map_preprocessed2.pcd \
  aligned_cloud_publish_rate_hz:=0.0 \
  path_publish_interval_s:=0.0 \
  grid_visualization_rate_hz:=20.0 \
  start_localization_rviz:=true \
  start_local_planner_rviz:=true
```

For an onboard before/after measurement, run
`bash scripts/profile_navigation_cpu.sh 60` while following the same route and
using the same map and sensor rates. See `docs/performance.md` for the acceptance
method and limitations of the current workstation-only validation.

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

## Multi-waypoint missions

The integrated launch can read an ordered waypoint list from a ROS 2 parameter YAML. The
mission manager publishes only the current waypoint to FastPlanner. It waits until FastPlanner
publishes a path whose endpoint matches that waypoint, and then waits for odometry to remain
within the configured arrival tolerance before publishing the next waypoint. Every leg
therefore continues to use the full FastPlanner A* -> SCAN-Planner -> controller chain; later
waypoints cannot overwrite an earlier waypoint before it is planned and reached.

Create a YAML file using numeric `x, y, z` triples in the `map` frame:

```yaml
waypoint_mission_manager:
  ros__parameters:
    waypoints: [
      1.0, 0.0, 0.3,
      2.0, 1.0, 0.3,
      3.0, 0.0, 0.3
    ]
```

An editable example is installed from
`src/navigation_bringup/config/waypoints.example.yaml`. Start the mission with an absolute
YAML path:

```bash
ros2 launch navigation_bringup navigation_system.launch.py \
  map_pcd_path:=$PWD/maps/map_preprocessed2.pcd \
  waypoints_file:=$PWD/src/navigation_bringup/config/waypoints.example.yaml
```

Useful mission arguments are:

- `waypoint_xy_tolerance` (default `0.5` m): arrival radius in the XY plane.
- `waypoint_z_tolerance` (default `-1.0`): negative disables the Z arrival check.
- `waypoint_path_goal_tolerance` (default `0.75` m): allowed XY error when confirming that
  FastPlanner produced a path for the current waypoint.
- `waypoint_hold_time` (default `0.5` s): required continuous time inside the tolerance.
- `waypoint_loop` (default `false`): repeat the route after the final waypoint.

The transient-local `/waypoint_mission/status` topic publishes compact JSON status messages.
If `waypoints_file` is omitted, the mission manager is not started and interactive single-goal
navigation behaves as before.

## 本窗口问题汇总（2026-07-24）

### 已分析的问题

- 已审计最小数据链路：FastAnchor 通过 `/fast_anchor/odom` 和
  `/fast_anchor/aligned_cloud` 提供定位，FastPlanner 接收定位和目标并发布
  `/planned_path`，SCAN-Planner 接收路径和实时点云并发布 `/planning/bspline`，
  控制器最终发布 `/cmd_vel`。当前局部规划器确认为 SCAN-Planner，不是 EGO。
- 已核对可视化职责：定位与 SCAN-Planner RViz 用于日常运行，
  `pct_global_planner.rviz` 只提供全局规划调试显示，正常运行不必默认打开。
- 已核对 SCAN 栅格实现与原项目。`grid_map.cpp` 和 `planner.yaml` 曾通过 SHA256
  与原始 SCAN-Planner 对比一致；RViz 中只保留集成需要的 frame 和 topic 修改。
- 已分析局部目标过远问题。FastPlanner 发布完整路径，SCAN 实际通过
  `fsm.planning_horizon` 从路径上选局部目标；原值 7.5 m 大于 5.0 m 的局部感知范围。
- 已分析障碍高度参数。`grid_map.obstacles_inflation_z_up` 是障碍物向上膨胀量，
  它只改变碰撞模型，不会提升 Go2 的真实抬腿或越障能力。
- 已按要求执行在线诊断，且诊断前先执行了 `source setup.bash`。定位位姿和配准点云
  均约 10 Hz，在线膨胀点云约 3.8 Hz，控制命令约 100 Hz；FastPlanner 状态为
  `SUCCESS`，局部规划失败发生在 SCAN 内部 A* 和 B-spline 优化阶段。

### 已解决的问题

- 已完成三个独立 ROS 2 包之间的最小接口连接，不需要额外
  `navigation_adapter`；原始目录未被修改。
- 定位和 SCAN-Planner 可视化默认开启；FastPlanner 独立调试 RViz 默认关闭，
  可通过 `start_global_planner_rviz:=true` 临时开启。
- 已加入 `lidar_model` 启动参数，支持 `mid360` 与 `mid360s`；集成启动默认值为
  `mid360s`，本次在线运行实际使用 `lidar_model:=mid360`。
- SCAN 的普通占用云和膨胀云发布逻辑已恢复为原版，并使用 Release 模式构建。
  隔离测试中输入点云为 10 Hz 时，`/grid_map/occupancy` 和
  `/grid_map/occupancy_inflate` 均达到约 20 Hz。
- 已加入 `local_target_distance` 参数，默认 4.0 m；本次在线进程显式使用 3.0 m，
  运行时 `fsm.planning_horizon` 已确认是 3.0 m。
- 已按“规划模型允许 40 cm 低矮障碍”的要求设置：
  `grid_map.body_height=0.50`、`grid_map.obstacles_inflation_z_up=0.10`、
  `obstacle_min_relative_z=0.40`。相关包构建成功，SCAN 启动测试 2/2 通过。
- ROS 2 CLI daemon 的 `rclpy.ok()` 异常已通过重启 daemon 恢复，不影响正在运行的
  导航节点。

### 尚未解决的问题

- **SCAN 在线局部规划仍失败。** 当前日志持续出现
  `The robot is inside an obstacle`、`A-star failed; aborting optimization` 和
  `Ran out of pool`。在线点云检查显示机器人当前位置附近没有占用点，错误来自
  初始轨迹控制点而非机器人当前体素。
- **全局路径和 SCAN 的 Z 语义不一致。** FastPlanner 当前路径首点已经使用机体
  定位高度，例如约 0.323 m；SCAN 在
  `scan_replan_fsm.cpp::pathCallback()` 中又固定增加 `body_height=0.50 m`，使对应
  waypoint 变成约 0.823 m。在线观察到 SCAN 全局参考轨迹一度上升到约 0.93 m，
  进入障碍占用层并触发局部 A* 失败。建议下一步让 FastPlanner 始终输出地面高度
  路径，保留 SCAN 原有的 `+ body_height`，以避免修改局部规划算法。
- FastAnchor 启动后定位 Z 曾从约 -1.35 m 变化到正值，定位高度尚未稳定时
  FastPlanner 已开始重规划。尚未增加定位稳定等待或目标门控逻辑。
- “40 cm”目前只是规划器碰撞判定阈值，没有加入爬楼梯步态、足端轨迹或抬腿控制，
  也没有完成真机 40 cm 越障测试。单个 40 cm 垂直障碍仍存在严重碰撞和跌倒风险。
- SCAN 的 GPU 渲染节点属于仿真传感器渲染，并不是实机膨胀点云可视化；当前实机
  bringup 未启用该 GPU 仿真节点。
- 在线整机负载下栅格可视化约 3.8 Hz，低于隔离 Release 测试的约 20 Hz，尚未做
  进一步性能修改，以保持 SCAN 原始发布逻辑。
- 在线日志发现 `/move_base_simple/goal` 存在一个 QoS 不兼容发布者；另一个兼容
  发布者仍能触发全局规划，因此不是本次局部 A* 失败的直接原因，但尚未统一 QoS。
- 当前 `maps/` 中实际存在 `map_preprocessed1.pcd` 与 `map_preprocessed2.pcd`，没有
  `map_preprocessed.pcd`。本次成功在线启动使用的是 `map_preprocessed2.pcd`，启动时
  应传入实际存在的地图文件。

当前在线启动示例：

```bash
cd <workspace>
source setup.bash
ros2 launch navigation_bringup navigation_system.launch.py \
  map_pcd_path:=$PWD/maps/map_preprocessed2.pcd \
  local_target_distance:=3.0 \
  lidar_model:=mid360
```

同一命令的一行形式：

```bash
ros2 launch navigation_bringup navigation_system.launch.py   map_pcd_path:=$PWD/maps/map_preprocessed2.pcd   local_target_distance:=3.0 lidar_model:=mid360 lio_backend:=yifanlio
```


ros2 launch navigation_bringup navigation_system.launch.py \
  map_pcd_path:=$PWD/maps/map_preprocessed2.pcd \
  runtime_log_map_target_rate_hz:=10.0 \
  runtime_log_report_interval_sec:=5.0 \
  runtime_log_csv_path:=/tmp/navigation-runtime.csv

停止导航
  ros2 service call /scan_planner/set_navigation_enabled std_srvs/srv/SetBool "{data: false}"
启动导航
  ros2 service call /scan_planner/set_navigation_enabled std_srvs/srv/SetBool "{data: true}"