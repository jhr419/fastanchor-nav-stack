# FastPlanner

FastPlanner 是从 3dnav 提取的独立 ROS 2 Humble 三维全局规划工作区。它保留
`astar / pct / jie_octomap` 三个后端，并在统一输出前加入从 MapProcessor
planner test 迁移并扩展的路径可行性闸门。

数据流：

```text
MapProcessor PCD + tomogram + BT
  -> 规划前 start/goal 验证（pre/both）
  -> astar / pct / jie_octomap
  -> /fast_global_planner/raw_path
  -> 逐点/逐段 traversability + collision + clearance 验证（post/both）
  -> /planned_path + /path + /planned_path_marker
```

## 独立构建

```bash
cd /path/to/FastPlanner
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

构建和运行不需要 source 其他工作空间的 `install/setup.bash`。默认寻找同级目录
`../MapProcessor`；非同级部署时设置：

```bash
export MAP_PROCESSOR_ROOT=/path/to/MapProcessor
```

## 启动

```bash
ros2 launch fast_global_planner global_planner.launch.py algorithm:=astar
ros2 launch fast_global_planner global_planner.launch.py algorithm:=pct
ros2 launch fast_global_planner global_planner.launch.py algorithm:=jie_octomap
```

常用覆盖：

```bash
ros2 launch fast_global_planner global_planner.launch.py \
  algorithm:=astar \
  feasibility_check_mode:=both \
  feasibility_clearance_threshold:=0.2 \
  fail_on_infeasible_path:=true \
  launch_rviz:=true
```

启动日志会打印：

```text
Feasibility check: ENABLED / DISABLED
Mode: pre / post / both
```

## 输入和输出

目标输入与原系统一致：

```text
/goal_pose_3d   geometry_msgs/msg/PoseStamped
/goal_point_3d  geometry_msgs/msg/PointStamped
/goal_pose      geometry_msgs/msg/PoseStamped
```

默认起点来自 `map -> base_link` TF。测试时可用 `start_source:=topic` 并发布：

```text
/fast_global_planner/start_pose  geometry_msgs/msg/PoseStamped
```

稳定输出：

```text
/planned_path         nav_msgs/msg/Path
/path                 nav_msgs/msg/Path
/planned_path_marker  visualization_msgs/msg/Marker
```

因此 3dnav 的局部规划器仍可直接订阅 `/planned_path`，无需修改消息合同。

## 验证

```bash
bash scripts/run_feasibility_test.sh --clearance-threshold 0.2
ros2 topic echo /fast_global_planner/feasibility_status
ros2 topic echo /planned_path --once
```

报告位于：

```text
debug/feasibility/feasibility_report.yaml
debug/feasibility/feasibility_points.csv
```

详细原理与调试方法见 [docs/feasibility_check.md](docs/feasibility_check.md)，
迁移审查见 [docs/migration_audit.md](docs/migration_audit.md)。

## PCT 原生兼容说明

原生 PCT pybind 模块依赖 Python 3.10 / NumPy 1.x ABI。FastPlanner 已包含原规划
运行库，`algorithm:=pct` 默认运行原生 PCT、优化与原路径后处理。需要排查二进制
兼容问题时，可显式切换到可移植多层 tomogram A*：

```bash
ros2 launch fast_global_planner global_planner.launch.py \
  algorithm:=pct use_native_pct_backend:=false
```

原生模式启动时，launch 会从 MapProcessor pickle 生成项目内
`debug/native_pct/*_numpy_compat.npz`，让 NumPy 1.x 原生节点读取同一地图内容，
不会修改或复制回 MapProcessor。

`astar` 默认后端及 `jie_octomap` 不依赖这组 Python pybind 模块。
