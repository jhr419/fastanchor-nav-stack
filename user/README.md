# FastAnchor 导航工程使用说明

本文件是工程环境配置、编译、启动、停止、测试和数据操作的唯一用户入口。所有命令均从工作空间根目录执行，脚本自身不依赖当前目录。

## 1. 系统组成

```text
Livox MID360/MID360s
  -> FAST-LIO2 或 yifanLIO
  -> FastAnchor 定位
  -> FastPlanner A* 全局规划
  -> SCAN-Planner 局部规划与闭环控制
  -> /cmd_vel
  -> 智身 L1 或 Unitree Go2 速度桥
```

默认运行配置位于 `user/config.env`：

- ROS 2：Humble
- 地图：`maps/map_preprocessed2.pcd`
- 雷达：MID360s
- LIO：FAST-LIO2
- 局部目标距离：`4.0 m`
- 底盘：智身 L1

## 2. 环境要求

- Ubuntu 图形桌面；多终端启动需要 `gnome-terminal`。
- ROS 2 Humble，安装路径为 `/opt/ros/humble`。
- `colcon`、RViz2 和工程声明的 ROS 依赖。
- Livox 雷达与主机网络已按设备配置完成。
- 使用智身 L1 或 Go2 时，底盘网络、急停和遥控接管必须可用。

无图形桌面的设备可使用单模块脚本，不使用 `start_all.sh`。

## 3. 脚本清单

| 脚本 | 类型 | 作用 |
|---|---|---|
| `setup_env.sh` | 环境 | 加载 ROS 2 与本工作空间环境 |
| `check_env.sh` | 检查 | 检查命令、安装空间、默认地图和底盘配置 |
| `build.sh` | 编译 | 以 Release 模式编译工程 |
| `start.sh` | 默认启动 | 调用多终端完整启动入口 |
| `start_all.sh` | 多终端 | 按依赖顺序启动完整系统 |
| `start_sensor.sh` | 单模块 | 启动 Livox 雷达驱动 |
| `start_localization.sh` | 单模块 | 等待雷达后启动 LIO 与 FastAnchor |
| `start_planner.sh` | 单模块 | 等待定位后启动规划与控制 |
| `start_navigation.sh` | 单终端 | 一体化启动传感器、定位、规划与控制，不启动底盘桥 |
| `start_zs_bridge.sh` | 单模块 | 等待 `/cmd_vel` 后启动智身 L1 速度桥 |
| `start_go2_bridge.sh` | 单模块 | 等待 `/cmd_vel` 后启动 Go2 速度桥 |
| `start_rviz.sh` | 调试 | 等待定位后加载固定 RViz 配置 |
| `stop.sh` | 停止 | 停止由上述脚本登记的进程 |
| `test.sh` | 测试 | 运行核心包回归测试或全工程测试 |
| `record.sh` | 数据 | 录制导航核心 Topic |
| `play_bag.sh` | 数据 | 回放指定 ROS 2 Bag |

新增或修改脚本时必须同步更新本表。

## 4. 第一次使用

```bash
./user/build.sh
./user/check_env.sh --robot zs
```

加载环境到当前终端：

```bash
source user/setup_env.sh
```

根目录的 `setup.bash` 是兼容入口，效果等同于 `source user/setup_env.sh`。

## 5. 编译

完整编译：

```bash
./user/build.sh
```

只编译指定包及其依赖：

```bash
./user/build.sh --packages-up-to navigation_bringup genisom_l1_control
```

构建完成后，重新打开终端或执行 `source user/setup_env.sh`。

## 6. 完整系统启动

默认启动 MID360s、FAST-LIO2、导航系统和智身 L1 速度桥：

```bash
./user/start.sh
```

常用参数：

```bash
./user/start.sh \
  --robot zs \
  --map maps/map_preprocessed2.pcd \
  --lidar-model mid360s \
  --lio-backend fastlio2 \
  --local-target-distance 4.0 \
  --rviz
```

可选值：

- `--robot zs|go2|none`
- `--lidar-model mid360|mid360s`
- `--lio-backend fastlio2|yifanlio`
- `--rviz` 或 `--no-rviz`

额外的 ROS launch 参数放在 `--` 后面：

```bash
./user/start.sh --robot zs -- runtime_log_enabled:=false
```

在不启动进程的情况下检查终端顺序和命令：

```bash
./user/start_all.sh --dry-run --robot zs --rviz
```

### 多终端顺序

| 编号 | 终端 | 启动条件 |
|---|---|---|
| 01 | Livox MID360/MID360s | 立即启动 |
| 02 | LIO 与 FastAnchor 定位 | 等待 `/livox/lidar`、`/livox/imu` |
| 03 | FastPlanner、SCAN 与控制 | 等待 `/fast_anchor/odom`、`/fast_anchor/aligned_cloud` |
| 04 | 智身 L1 或 Go2 速度桥 | 等待 `/cmd_vel` |
| 05 | 导航 RViz，可选 | 等待 `/fast_anchor/odom` |

等待默认不超时。需要无人值守失败退出时，可在 `user/config.env` 中把 `NAV_TOPIC_WAIT_TIMEOUT` 设置为正整数秒数。

## 7. 单模块启动

单终端一体化导航，不连接底盘：

```bash
./user/start_navigation.sh --map maps/map_preprocessed2.pcd
```

拆分启动时，在不同终端依次执行：

```bash
./user/start_sensor.sh --lidar-model mid360s
./user/start_localization.sh --map maps/map_preprocessed2.pcd --lio-backend fastlio2
./user/start_planner.sh --map maps/map_preprocessed2.pcd
./user/start_zs_bridge.sh
```

Go2 使用：

```bash
./user/start_go2_bridge.sh
```

启动可视化：

```bash
./user/start_rviz.sh
```

所有依赖模块脚本都会自行等待所需 Topic，不要求用户估计启动时间。

## 8. 停止

```bash
./user/stop.sh
```

脚本首先发送 `SIGINT`，给 ROS 2 launch 最多 10 秒完成正常退出；仍未退出时发送 `SIGTERM`。它只处理 `user/*.sh` 登记且进程启动时间匹配的 PID，不会使用宽泛的 `pkill` 停止其他 ROS 工程。

如果节点不是通过本工程脚本启动，应在对应终端使用 `Ctrl+C`。

## 9. 测试

运行定位、规划、控制、任务管理和智身速度桥的核心回归测试：

```bash
./user/test.sh
```

运行指定包：

```bash
./user/test.sh nav3d_global_planning scan_planner
```

运行全工作空间测试：

```bash
./user/test.sh --all
```

测试结束后脚本自动执行 `colcon test-result --verbose`，存在失败时返回非零状态。

## 10. 数据录制与回放

开始录制导航核心 Topic：

```bash
./user/record.sh
```

默认输出到 `bags/navigation_日期_时间`。指定相对输出目录：

```bash
./user/record.sh --output bags/site_a_run_01
```

回放：

```bash
./user/play_bag.sh bags/site_a_run_01
```

向 `ros2 bag play` 传递额外参数：

```bash
./user/play_bag.sh bags/site_a_run_01 --clock --rate 0.5
```

## 11. 常用接口

| Topic / Service | 作用 |
|---|---|
| `/livox/lidar` | Livox 点云 |
| `/livox/imu` | Livox IMU |
| `/fast_anchor/odom` | 地图坐标系定位 |
| `/fast_anchor/aligned_cloud` | 配准点云 |
| `/move_base_simple/goal` | 单目标输入 |
| `/planned_path` | A* 全局路径 |
| `/planning/bspline` | SCAN 轨迹 |
| `/cmd_vel` | 标准 ROS 速度指令 |
| `/astar_global_planner/status` | A* 状态，到达后为 `GOAL_REACHED` |
| `/scan_planner/set_navigation_enabled` | 启用或停止 SCAN 执行 |

## 12. 常见问题

### 地图不存在

脚本在启动前检查地图。使用仓库内实际存在的相对路径，或修改 `user/config.env` 的 `NAV_DEFAULT_MAP`。

### 终端一直显示“等待 Topic”

先检查前一个编号终端是否报错，再执行：

```bash
source user/setup_env.sh
ros2 topic list
```

不要跳过依赖等待直接启动后级模块。

### `gnome-terminal` 无法启动

确认当前处于图形桌面会话。SSH 或无桌面设备使用 `start_navigation.sh` 和单模块脚本。

### 提示模块已经运行

先执行 `./user/stop.sh`。脚本会自动清理已经失效的 PID 记录。

### 智身 L1 不接受控制

检查主机和 `192.168.168.168` 的网络连接、SDK 端口、遥控器控制权以及 `twist.yaml`。不要通过反复重启自动抢占遥控器控制权。

### Go2 无法通信

确认 Unitree 网络接口配置，并保证所有终端使用相同的 `ROS_DOMAIN_ID`。Go2 脚本会自动设置 `rmw_cyclonedds_cpp`。

## 13. 修改同步要求

任何影响启动命令、Launch、Topic、Node、参数、环境、启动顺序或依赖关系的修改，都必须同步检查：

```text
代码 -> user 脚本 -> user/README.md -> 测试
```

脚本中只放工程管理和操作逻辑，业务实现继续放在 `src/` 对应 ROS 包内。
