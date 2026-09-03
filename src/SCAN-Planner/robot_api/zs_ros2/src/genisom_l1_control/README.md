# genisom_l1_control

ROS2 Humble C++ 包，提供完整 `genisom_manager_node` 和精简 `genisom_twist_node` 两个部署目标。
Manager 仍使用新版 `ZsibotExecutor` 承担遥测、控制权和服务；Twist 控制使用旧版 ZSL-1W
`HighLevel::move(vx, vy, yaw_rate)` 直接下发 `/cmd_vel` 物理速度。

## 构建与启动

从工作区根目录执行：

```bash
source /opt/ros/humble/setup.bash
./scripts/build_all.sh
source install/setup.bash
ros2 launch genisom_l1_control manager.launch.py
```

只需要接收 `/cmd_vel` 并直接控制 ZSL-1W 运动时：

```bash
ros2 launch genisom_l1_control twist.launch.py
```

## 组件

| 组件 | 说明 |
|---|---|
| `ManagerNode` | ROS service/topic/parameter、安全状态机和数据转换 |
| `SdkWrapper` | Manager 的新版 SDK 实例、调用串行化和型号策略 |
| `HighLevelVelocityClient` | Twist 的旧版 ZSL-1W highlevel 速度客户端 |
| `ProcessSupervisor` | 官方 SDK 构造卡死时保证 Ctrl+C 可回收 |
| `manager.yaml` | 网络、频率、发布开关、frame、速度映射和可选限幅 |
| `TwistNode` | 通过旧版 highlevel 直接执行 `/cmd_vel` 的 x/y/yaw 速度 |
| `twist.yaml` | Twist highlevel 网络、站立流程和速度限幅配置 |
| `manager.launch.py` | 正式启动入口 |
| `twist.launch.py` | 独立 Twist 控制入口 |
| `control.launch.py` | 兼容旧启动命令，仍启动 Manager |

## 开发约束

- 使用 C++20，与官方根 CMake 的真实配置一致；
- 注释使用中文，项目路径使用相对路径；
- 不修改 `third_party/genisom_L1_sdk`；
- Manager 的新版 SDK 调用必须经过 `SdkWrapper`；
- Twist 的旧版 highlevel 速度调用必须经过 `HighLevelVelocityClient`；
- Manager 控制权以 `GetFunctionMode()` 反馈为准；
- Manager 只有 `FM_SDK` 允许速度桥；
- 型号限制必须来自官方仓库，不猜测；
- 不把 LOCK 解释成阻尼，不虚构 clear ESTOP、时间戳、协方差或电机名称。

完整用户文档见 `../../docs/USER_GUIDE.md`。

## Twist 运行约束

- Twist 使用 `third_party/genisom_l1_sdk_old` 中的旧版 ZSL-1W highlevel 动态库。
- 机器人端 `/opt/export/config/sdk_config.yaml` 的 `target_ip/target_port` 必须与 `local_ip/local_port` 一致。
- 收到一条 `/cmd_vel` 立即调用一次 `HighLevel::move(linear.x, linear.y, angular.z)`。
- 不再把 ROS 速度乘摇杆增益，不再使用 `SetRemote()` 发送速度。
- `linear.x`、`linear.y` 和 `angular.z` 同时可用，不做 yaw 优先或 x/yaw 互斥。
- 不再固定频率持续发送速度或零速，也不再启用 300 ms 超时清零看门狗。
- 旧版 highlevel 接口没有新版 `FunctionMode` 反馈，Twist 状态不再报告遥控器接管确认。
