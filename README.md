# fastanchor-nav-stack 使用说明

本仓库把 FastAnchor 定位、FAST-LIO2 前端、FastPlanner 全局规划、SCAN-Planner 局部规划和 Unitree Go2 控制接口集成到一个 ROS 2 Humble 工作区中。

默认运行链路：

```text
Livox MID360/MID360s + IMU
  -> FAST-LIO2 /Odometry
  -> FAST-LIO2 与 Unitree 腿里程计融合
  -> FastAnchor ICP 地图定位
  -> FastPlanner A* 全局路径
  -> SCAN-Planner 局部轨迹
  -> /cmd_vel
```

## 1. 构建

首次构建：

```bash
cd ~/hy_ws/fastanchor-nav-stack
source /opt/ros/humble/setup.bash
source setup.bash
colcon build --symlink-install
source install/setup.bash
```

只重新构建定位和导航相关包：

```bash
cd ~/hy_ws/fastanchor-nav-stack
source /opt/ros/humble/setup.bash
source setup.bash
source install/setup.bash
colcon build --symlink-install --packages-select \
  fast_anchor_fusion fast_anchor_localization fast_anchor_bringup navigation_bringup
source install/setup.bash
```

如果从另一台机器复制了整个工作区，并且编译时报 `CMakeCache.txt directory is different`，说明 `build/`、`install/`、`log/` 中带了旧机器路径。清掉后重新构建：

```bash
cd ~/hy_ws/fastanchor-nav-stack
rm -rf build install log
source /opt/ros/humble/setup.bash
source setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 2. 从本机复制到 Go2

在本机执行：

```bash
cd ~/ros_ws
tar --exclude='fastanchor-nav-stack/build' \
    --exclude='fastanchor-nav-stack/install' \
    --exclude='fastanchor-nav-stack/log' \
    -czf fastanchor-nav-stack.tar.gz fastanchor-nav-stack/

scp fastanchor-nav-stack.tar.gz go2@192.168.123.99:~/hy_ws/
```

在 Go2 上执行：

```bash
ssh go2@192.168.123.99
cd ~/hy_ws
tar -xzf fastanchor-nav-stack.tar.gz
cd fastanchor-nav-stack
source /opt/ros/humble/setup.bash
source setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 3. 启动导航系统

默认参数来自：

```text
src/navigation_bringup/config/navigation_system.yaml
```

启动：

```bash
cd ~/hy_ws/fastanchor-nav-stack
source /opt/ros/humble/setup.bash
source setup.bash
source install/setup.bash

ros2 launch navigation_bringup navigation_system.launch.py
```

常用启动覆盖参数：

```bash
ros2 launch navigation_bringup navigation_system.launch.py \
  map_pcd_path:=$PWD/maps/map_preprocessed2.pcd \
  lidar_model:=mid360 \
  odometry_fusion_mode:=leg \
  local_target_distance:=3.0
```

重要说明：

- `map_pcd_path` 必须指向真实存在的 PCD 地图。
- `lidar_model` 可选 `mid360` 或 `mid360s`。
- `odometry_fusion_mode:=leg` 会启用 FAST-LIO2 与 Unitree 腿里程计融合。
- `odometry_fusion_mode:=none` 会让 FastAnchor 直接使用 FAST-LIO2 输出。
- `/fast_anchor/odom` 需要收到初始位姿后才会稳定输出。

## 4. 给定位初值

推荐使用 RViz 的 `2D Pose Estimate`，发布到：

```text
/initialpose
```

命令行方式：

```bash
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
"{header: {frame_id: 'map'}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}, covariance: [0.25, 0, 0, 0, 0, 0, 0, 0.25, 0, 0, 0, 0, 0, 0, 0.25, 0, 0, 0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0, 0, 0.1]}}"
```

重置定位后重新给初值：

```bash
ros2 service call /fast_anchor_reset_localization \
  fast_anchor_interfaces/srv/ResetLocalization \
  "{reset_to_initial_pose: false}"
```

## 5. 关键话题检查

启动后另开终端：

```bash
cd ~/hy_ws/fastanchor-nav-stack
source /opt/ros/humble/setup.bash
source setup.bash
source install/setup.bash

ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
ros2 topic hz /Odometry
ros2 topic hz /leg_odom
ros2 topic hz /fast_anchor/fusion/filtered_base_odom
ros2 topic hz /fast_anchor/odom
```

正常情况下：

| 话题 | 作用 | 典型频率 |
|---|---|---|
| `/livox/lidar` | Livox 点云 | 约 10 Hz |
| `/livox/imu` | Livox IMU | 约 200 Hz |
| `/Odometry` | FAST-LIO2 前端里程计 | 约 10 Hz |
| `/leg_odom` | Unitree 高层状态转标准里程计 | 约 20 Hz |
| `/fast_anchor/fusion/filtered_base_odom` | EKF 融合后的 base_link 里程计 | 约 50 Hz |
| `/fast_anchor/odom` | FastAnchor 地图定位输出 | 给初值后输出 |

定位状态：

```bash
ros2 topic echo /fast_anchor/status
ros2 topic echo /fast_anchor/icp_result
```

## 6. 单点导航

给一个 `map` 坐标系下的目标点：

```bash
ros2 topic pub --once /move_base_simple/goal geometry_msgs/msg/PoseStamped \
"{header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.0, z: 0.3}, orientation: {w: 1.0}}}"
```

导航数据流：

```text
/move_base_simple/goal
  -> /planned_path
  -> /scan_planner/initial_path
  -> /planning/bspline
  -> /cmd_vel
```

暂停局部规划和控制：

```bash
ros2 service call /scan_planner/set_navigation_enabled \
  std_srvs/srv/SetBool "{data: false}"
```

恢复局部规划和控制：

```bash
ros2 service call /scan_planner/set_navigation_enabled \
  std_srvs/srv/SetBool "{data: true}"
```

## 7. Action 多点任务

Action 名称：

```text
/follow_waypoints
```

Action 类型：

```text
nav_interfaces/action/FollowWaypoints
```

字段定义：

```text
Goal:
  string mission_id
  string frame_id
  geometry_msgs/Point[] waypoints

Result:
  bool success
  string message

Feedback:
  uint32 current_index
  uint32 total_waypoints
  float32 progress
  string state
```

发送多点任务：

```bash
ros2 action send_goal /follow_waypoints nav_interfaces/action/FollowWaypoints \
"{mission_id: 'demo_route', frame_id: 'map', waypoints: [{x: 1.0, y: 0.0, z: 0.3}, {x: 2.0, y: 0.5, z: 0.3}, {x: 1.0, y: 0.0, z: 0.3}]}" \
--feedback
```

查看任务状态：

```bash
ros2 topic echo /waypoint_mission/status
```

状态消息是 JSON 字符串，常见状态包括：

| 状态 | 含义 |
|---|---|
| `IDLE` | 没有任务 |
| `STARTING` | 任务启动中 |
| `WAITING_FOR_ODOMETRY` | 等待定位里程计 |
| `WAITING_FOR_PATH` | 已发布当前目标，等待全局路径确认 |
| `NAVIGATING` | 正在执行当前路点 |
| `PAUSING` | 暂停请求处理中 |
| `PAUSED` | 已暂停 |
| `RESUMING` | 恢复请求处理中 |
| `COMPLETED` | 路点全部完成 |

暂停当前任务：

```bash
ros2 service call /waypoint_mission/control nav_interfaces/srv/ControlMission \
"{mission_id: 'demo_route', command: 1}"
```

恢复当前任务：

```bash
ros2 service call /waypoint_mission/control nav_interfaces/srv/ControlMission \
"{mission_id: 'demo_route', command: 2}"
```

`mission_id` 可以留空，此时控制当前任务：

```bash
ros2 service call /waypoint_mission/control nav_interfaces/srv/ControlMission \
"{mission_id: '', command: 1}"
```

## 8. 参数在哪里改

| 功能 | 配置位置 | 常改参数 |
|---|---|---|
| 集成启动默认值 | `src/navigation_bringup/config/navigation_system.yaml` | 地图、雷达型号、话题名、是否启动各模块 |
| FastAnchor ICP 定位 | `src/FastAnchor/src/fast_anchor_bringup/config/fast_anchor_localization.yaml` | ICP 阈值、修正门控、点云过滤 |
| FAST-LIO2 | `src/FastAnchor/src/third_party/FAST_LIO/config/mid360_localization.yaml` | 雷达/IMU 话题、外参、滤波范围 |
| Livox 驱动 | `src/FastAnchor/src/third_party/livox_ros_driver2/config/MID360_config.json` | 雷达 IP、主机 IP |
| Livox MID360s 驱动 | `src/FastAnchor/src/third_party/livox_ros_driver2/config/MID360s_config.json` | 雷达 IP、主机 IP |
| A* 全局规划 | `src/FastPlanner/src/3dnav_global_planning/config/astar_global_planner.yaml` | 分辨率、障碍膨胀、搜索参数 |
| SCAN 局部规划 | `src/SCAN-Planner/planner/plan_manage/config/planner.yaml` | 局部地图、规划 horizon、碰撞参数 |
| 控制器 | `src/SCAN-Planner/planner/plan_manage/config/controllers.yaml` | 速度限制、跟踪控制参数 |

常用 launch 参数：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `map_pcd_path` | `../../../maps/map_preprocessed2.pcd` | 地图 PCD |
| `lidar_model` | `mid360` | `mid360` 或 `mid360s` |
| `odometry_fusion_mode` | `leg` | `leg` 启用腿里程计融合，`none` 直接用 FAST-LIO2 |
| `unitree_sportmode_topic` | `/lf/sportmodestate` | Unitree 高层状态 |
| `leg_odom_topic` | `/leg_odom` | 转换后的腿里程计 |
| `raw_fast_lio_odom_topic` | `/Odometry` | FAST-LIO2 原始里程计 |
| `fused_body_odom_topic` | `/fast_anchor/fusion/fused_body_odom` | 给 FastAnchor 使用的融合 body 里程计 |
| `localization_pose_topic` | `/fast_anchor/odom` | 地图定位输出 |
| `goal_topic` | `/move_base_simple/goal` | 单点目标输入 |
| `global_path_topic` | `/planned_path` | 全局路径输出 |
| `cmd_vel_topic` | `/cmd_vel` | 速度命令 |
| `local_target_distance` | `4.0` | SCAN 从全局路径上截取局部目标的距离 |
| `start_localization_rviz` | `false` | 是否启动定位 RViz |
| `start_global_planner_rviz` | `false` | 是否启动全局规划 RViz |
| `start_local_planner_rviz` | `false` | 是否启动 SCAN RViz |

## 9. 定位稳定性保护

当前默认开启两层保护。

第一层在 `fast_anchor_odom_frame_adapter` 中，配置位于：

```text
src/navigation_bringup/config/navigation_system.yaml
```

参数块：

```yaml
fast_anchor_odom_frame_adapter:
  ros__parameters:
    consistency_check:
      enabled: true
      max_translation_error_m: 0.35
      max_yaw_error_rad: 0.35
      max_lio_motion_when_leg_stationary_m: 0.18
      max_distance_ratio: 3.0
```

作用：短时间窗口内比较 FAST-LIO2 和 Unitree 腿里程计。如果 FAST-LIO2 明显飘，会提高 FAST-LIO2 协方差，让 EKF 降低它的权重。

第二层在 FastAnchor ICP 中，配置位于：

```text
src/FastAnchor/src/fast_anchor_bringup/config/fast_anchor_localization.yaml
```

参数块：

```yaml
icp:
  correction_gate:
    enabled: true
    warmup_accept_count: 3
    max_translation_m: 0.35
    max_rotation_rad: 0.35
    smoothing_alpha: 0.6
    recovery_enabled: true
    recovery_min_reject_count: 5
    recovery_stable_translation_m: 0.20
    recovery_stable_rotation_rad: 0.20
    recovery_smoothing_alpha: 0.25
```

作用：ICP fitness 合格但单次修正过大时，先拒绝这次修正，避免定位跳飞。如果连续多帧 ICP 都给出稳定的大修正，则进入 recovery，用较小 alpha 慢慢拉回地图。

调参建议：

| 现象 | 建议 |
|---|---|
| 正常运动也频繁 `FAST-LIO odometry marked unhealthy` | 增大 `max_translation_error_m` 或 `max_distance_ratio` |
| 静止时 FAST-LIO2 慢慢漂但没有触发 | 减小 `max_lio_motion_when_leg_stationary_m` |
| ICP 经常 `rejected by gate` 且无法恢复 | 增大 `max_translation_m` 或减小 `recovery_min_reject_count` |
| ICP 偶发把定位拉飞 | 减小 `max_translation_m`，或增大 `recovery_min_reject_count` |
| 定位被拉回太慢 | 增大 `recovery_smoothing_alpha` |

## 10. 录制 bag

静止测试：

```bash
ros2 bag record -o odom_static_test \
  /livox/lidar /livox/imu /Odometry /leg_odom \
  /fast_anchor/fusion/fast_lio_base_odom \
  /fast_anchor/fusion/filtered_base_odom \
  /fast_anchor/fusion/fused_body_odom \
  /fast_anchor/odom /fast_anchor/status /fast_anchor/icp_result /tf /tf_static
```

运动测试：

```bash
ros2 bag record -o odom_motion_test \
  /livox/lidar /livox/imu /Odometry /leg_odom \
  /fast_anchor/fusion/fast_lio_base_odom \
  /fast_anchor/fusion/filtered_base_odom \
  /fast_anchor/fusion/fused_body_odom \
  /fast_anchor/odom /fast_anchor/status /fast_anchor/icp_result /tf /tf_static
```

建议运动模式：

```text
静止 10 秒 -> 直走一小段 -> 原地转向 -> 回到起点附近 -> 静止 10 秒
```

## 11. 常见问题

只看到 `/parameter_events` 和 `/rosout`：

- 确认已经 `source setup.bash` 和 `source install/setup.bash`。
- 确认 CycloneDDS 使用的是连接 Go2 的网卡。
- 确认本机和 Go2 能互相 ping 通。

`/leg_odom` 没有数据：

```bash
ros2 topic hz /lf/sportmodestate
ros2 topic echo --once /lf/sportmodestate
```

如果 `/lf/sportmodestate` 有数据但 `/leg_odom` 没数据，检查：

```text
src/navigation_bringup/config/navigation_system.yaml
```

确认：

```yaml
unitree_sportmode_to_odom:
  ros__parameters:
    drop_on_error: false
```

`/fast_anchor/odom` 没有数据：

- 先确认 `/Odometry` 和 `/fast_anchor/fusion/fused_body_odom` 有数据。
- 再给 `/initialpose` 初值。
- 查看 `/fast_anchor/status` 当前状态。

ICP 一直失败：

- 检查地图是否和现场一致。
- 检查初值是否离真实位置太远。
- 检查 `cloud_preprocess.min_range`、自机过滤盒和地图点云是否过稀。

规划有路径但机器不动：

```bash
ros2 topic hz /planned_path
ros2 topic hz /scan_planner/initial_path
ros2 topic hz /planning/bspline
ros2 topic hz /cmd_vel
```

如果 `/cmd_vel` 有数据但机器不动，继续检查 Unitree 控制桥和机器狗运动模式。

## 12. 推荐 commit message

本次定位融合和 README 一起提交时推荐：

```bash
git add .
git commit -m "增强里程计融合稳定性并完善导航使用文档"
```

如果只提交代码，不包含 README：

```bash
git commit -m "增强FAST-LIO2里程计健康检查与ICP修正门控"
```

如果只提交 README：

```bash
git commit -m "完善导航系统使用说明"
```
