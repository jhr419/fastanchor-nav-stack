# genisom_l1_control

ROS2 Humble C++ 包，提供完整 `genisom_manager_node` 和精简 `genisom_twist_node` 两个互斥部署目标。
任一时刻只能运行其中一个节点并持有 GENISOM 官方 `ZsibotExecutor`。

## 构建与启动

从工作区根目录执行：

```bash
source /opt/ros/humble/setup.bash
./scripts/build_all.sh
source install/setup.bash
ros2 launch genisom_l1_control manager.launch.py
```

只需要自动进入 SDK 并接收 `/cmd_vel` 时：

```bash
ros2 launch genisom_l1_control twist.launch.py
```

## 组件

| 组件 | 说明 |
|---|---|
| `ManagerNode` | ROS service/topic/parameter、安全状态机和数据转换 |
| `SdkWrapper` | 唯一官方 SDK 实例、调用串行化、型号策略 |
| `ProcessSupervisor` | 官方 SDK 构造卡死时保证 Ctrl+C 可回收 |
| `manager.yaml` | 网络、频率、发布开关、frame、速度映射和可选限幅 |
| `TwistNode` | 自动进入 SDK、只执行前后/偏航速度、处理遥控器接管 |
| `twist.yaml` | Twist 控制权、模式切换、看门狗和轴映射配置 |
| `manager.launch.py` | 正式启动入口 |
| `twist.launch.py` | 独立 Twist 控制入口 |
| `control.launch.py` | 兼容旧启动命令，仍启动 Manager |

## 开发约束

- 使用 C++20，与官方根 CMake 的真实配置一致；
- 注释使用中文，项目路径使用相对路径；
- 不修改 `third_party/genisom_L1_sdk`；
- Manager 与 Twist 节点不能同时运行，整个部署始终只有一份 SDK 实例；
- 所有 SDK 调用必须经过 `SdkWrapper`；
- 控制权以 `GetFunctionMode()` 反馈为准；
- 只有 `FM_SDK` 允许速度桥；
- 型号限制必须来自官方仓库，不猜测；
- 不把 LOCK 解释成阻尼，不虚构 clear ESTOP、时间戳、协方差或电机名称。

完整用户文档见 `../../docs/USER_GUIDE.md`。
