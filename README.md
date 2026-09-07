# FastAnchor Navigation Stack

本工作空间集成 Livox、LIO、FastAnchor、FastPlanner、SCAN-Planner 以及智身 L1/Unitree Go2 底盘速度桥，提供从定位、规划到运动控制的 ROS 2 导航链路。

## 用户入口

环境配置、编译、启动、停止、测试和数据回放统一参见：

**[user/README.md](user/README.md)**

不要依赖根目录中的临时维护脚本或历史命令启动系统。

## 核心数据流

```text
Livox -> LIO -> FastAnchor -> FastPlanner A* -> SCAN-Planner -> /cmd_vel -> 底盘速度桥
```

| 模块 | 主要输入 | 主要输出 |
|---|---|---|
| FastAnchor | LIO 位姿、Livox 点云、初始位姿 | `/fast_anchor/odom`、`/fast_anchor/aligned_cloud` |
| FastPlanner A* | 定位、目标点、地图 | `/planned_path` |
| SCAN-Planner | 定位、配准点云、全局路径 | `/planning/bspline`、`/cmd_vel` |
| 底盘速度桥 | `/cmd_vel` | 智身 L1 或 Unitree Go2 SDK 指令 |

## 目录

| 目录 | 职责 |
|---|---|
| `src/` | ROS 2 包和业务源码 |
| `user/` | 唯一用户操作入口与工程管理脚本 |
| `maps/` | 定位和规划地图 |
| `docs/` | 架构、性能和问题记录 |
| `build/`、`install/`、`log/` | colcon 生成目录 |

## 开发文档

- [工程规范化设计](docs/engineering_standardization.md)
- [性能测试方法](docs/performance.md)
- [导航目标完成与释放](docs/scan_planner_goal_completion_release.md)
- [FastAnchor 架构](src/FastAnchor/docs/architecture.md)

修改启动命令、Launch、Topic、参数或依赖关系时，必须同步更新 `user/` 脚本、`user/README.md` 和相关测试。
