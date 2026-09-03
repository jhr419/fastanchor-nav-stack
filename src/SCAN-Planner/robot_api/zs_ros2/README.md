# GENISOM L1-W ROS2 Manager

面向 GENISOM L1-W 轮足机器人的 ROS2 Humble SDK 适配层。项目提供完整 Manager 和精简 Twist 控制两种
模式。Manager 使用新版 `ZsibotExecutor` 提供遥测和服务；Twist 模式参考旧版 ZSL-1W highlevel SDK，
通过 `HighLevel::move(vx, vy, yaw_rate)` 直接下发 `/cmd_vel` 物理速度。

完整使用说明见 `docs/USER_GUIDE.md`，SDK 依据见 `docs/SDK_ANALYSIS.md`，状态机设计见
`docs/CONTROL_ARCHITECTURE.md`，实机验收项目见 `docs/TEST_PLAN.md`。

## 核心原则

1. Manager 与 Twist 面向不同 SDK 后端，现场运行时仍建议二选一，避免多进程同时给运动控制发速度。
2. Manager 模式启动后默认不抢控制权，速度桥默认关闭。
3. Manager 只有在官方反馈 `FM_SDK` 且速度桥显式开启时才下发；Twist 模式要求 highlevel 已连接且状态为 `ACTIVE`。
4. 切回 REMOTE 前先关闭速度桥、停车、清除历史速度，再请求并确认遥控控制权。
5. 人工接管或断线重连后不自动抢回 SDK，不自动恢复旧速度。
6. 遥测与控制权解耦，REMOTE、SDK、GENERAL_SDK、ROAMERX 下都持续发布状态数据。
7. 型号动作限制来自官方 `GetModel()` 与当前仓库矩阵，不根据经验猜测。
8. `CMD_LOCK_MODE` 只称为锁定模式；官方没有阻尼接口，因此不实现阻尼模式。

## 官方 SDK 基线

- 仓库：`https://github.com/zsibot/genisom_L1_sdk`
- 分支：`main`
- 提交：`f7ccbf393e96f1205af8cce8070c5886f9641428`
- 提交日期：`2026-06-22T18:48:55+08:00`
- SDK 版本：根 CMake `1.1.0`，CHANGELOG `1.0.0`
- 本次开发前已执行 `git fetch --prune origin`，本地 HEAD 与 `origin/main` 一致

官方仓库 README 声明 C++17，但根 CMake 实际设置 C++20；本项目按真实构建配置使用 C++20。

## 已实现功能

- 控制权：REMOTE、SDK、GENERAL_SDK、ROAMERX 请求和反馈确认；
- 状态：站立、趴下、移动、平衡站立、锁定、实验室和文娱模式；
- 档位：慢速、正常、快速；
- 动作：匍匐、爬高台、卸货下蹲及官方其他动作服务；
- 型号保护：轮足自动拒绝跳跃、前跳、招手、后空翻、双腿站立；
- Manager 安全速度桥：显式 enable、可选 ROS 输入限幅、独立摇杆标定、SDK 协议边界和 300 ms watchdog；
- ESTOP：官方软急停和 Manager 锁存，恢复只释放 REMOTE；
- 遥测：IMU、odom、16 关节、电池、速度、电机温度、型号、版本、故障；
- 状态：`/genisom/status`、`/genisom/faults`、`/diagnostics`；
- SDK 构造卡死保护：监督进程在 Ctrl+C 后 3 秒强制回收工作进程；
- 独立 Twist 模式：启动后连接旧版 highlevel、可选站立，直接接收 `/cmd_vel`；
- Twist 速度下发：收到一条 `/cmd_vel` 立即调用一次 `HighLevel::move(linear.x, linear.y, angular.z)`；
- Twist 不再做 yaw 优先、x/yaw 互斥、固定频率重发或 300 ms 超时清零。

## 验证边界

| 项目 | 状态 |
|---|---|
| Manager 编译、launch/config 加载、GTest 和 lint | 已验证 |
| Twist 节点编译、launch/config 安装、highlevel 速度映射单测 | 已验证 |
| 官方 SDK 远端版本一致性 | 已验证 |
| 旧版验证工具连接 Firefly 并识别 `XGWHSPD / FM_REMOTE` | 已实机验证 |
| 新 Manager 的服务、遥测内容和动作效果 | 待 NUC 实机验证 |
| 控制权四模式切换、急停和断线恢复 | 待 NUC 实机验证 |
| `/cmd_vel` 方向、比例、odom/IMU/关节坐标 | 待实机标定 |
| 遥控器 `L2+R2+A` 两秒接管及 Twist 节点退出 | 待 NUC 实机验证 |

不能把“代码已实现或已编译”描述成“机器人已执行成功”。

## 构建

```bash
cd zs_sdk
source /opt/ros/humble/setup.bash
./scripts/build_all.sh
source install/setup.bash
```

测试：

```bash
colcon test --packages-select genisom_l1_control
colcon test-result --verbose
```

当前本地结果：

```text
69 tests, 0 errors, 0 failures, 13 skipped
```

跳过项来自 ROS2 Humble 对 cppcheck 2.7 的默认禁用策略。

## 启动

Manager 当前 Firefly 地址默认为 `192.168.168.168:8081`，NUC 本地监听 UDP `8080`。

```bash
ros2 launch genisom_l1_control manager.launch.py
```

查看状态：

```bash
ros2 topic echo /genisom/status
```

### 独立 Twist 控制模式

不需要 Manager 的遥测和动作服务，只需要直接执行 `/cmd_vel` 速度时，使用：

```bash
ros2 launch genisom_l1_control twist.launch.py
```

该启动方式会连接 `third_party/genisom_l1_sdk_old` 的 ZSL-1W highlevel 动态库，可选调用
`standUp()`，然后发布 `/genisom/twist/ready=true`。收到一条 `/cmd_vel` 就立即调用一次
`HighLevel::move(linear.x, linear.y, angular.z)`，不再走 `SetRemote()`、不再乘摇杆增益、不做
`linear.x/angular.z` 互斥，也不启用 300 ms 超时清零。

使用 Twist 前，需要确认机器人端 `/opt/export/config/sdk_config.yaml` 的 `target_ip/target_port` 与
`twist.yaml` 的 `local_ip/local_port` 一致。旧版 highlevel 接口没有新版 `FunctionMode` 反馈，因此 Twist
状态不再把遥控器接管作为可确认事件。

## 最小控制流程

已启动 Manager 时，可使用初始化脚本一次完成请求 SDK、站立、移动模式和开启速度桥：

```bash
./scripts/sdk_cmd.sh
```

脚本会要求输入 `1`。无人值守的明确授权场景可使用 `./scripts/sdk_cmd.sh --yes`。初始化成功后脚本显示数字
菜单并保持运行，上层可在另一个终端直接向 `/cmd_vel` 发布速度。常用映射为 `1` 趴下、`2` 站立、`3` 进入
移动模式并开启速度桥、`4` 平衡站立、`5` 锁定、`0` 锁存软件急停、`q` 安全切回 REMOTE。姿态 service
会关闭速度桥，需要继续行走时输入 `3`。脚本不区分慢速、正常和快速档位，速度由上层导航器或规划器管理。

`0` 不是姿态切换。触发后再次运行脚本会检测 ESTOP 锁存，要求操作员先按原厂流程确认恢复，再显式输入 `1`
调用 `/genisom/clear_emergency_stop`；脚本不会自动跳过急停锁存。

菜单各项、移动模式含义和典型流程详见 `docs/SDK_CMD_MENU_GUIDE.md`。

以下命令是对应的手工流程。

请求 SDK 并等待 `/genisom/status` 确认 `control_owner=SDK`：

```bash
ros2 service call /genisom/control/request_sdk std_srvs/srv/Trigger '{}'
```

设置机器人状态。每个命令都会先停车并关闭速度桥：

```bash
ros2 service call /genisom/mode/stand_up std_srvs/srv/Trigger '{}'
ros2 service call /genisom/mode/move std_srvs/srv/Trigger '{}'
```

显式开启速度桥：

```bash
ros2 service call /genisom/velocity_bridge/set_enabled std_srvs/srv/SetBool \
  '{data: true}'
```

发送极短低速指令：

```bash
ros2 topic pub --rate 10 --times 3 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.01, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

关闭速度桥并释放遥控器：

```bash
ros2 service call /genisom/velocity_bridge/set_enabled std_srvs/srv/SetBool \
  '{data: false}'
ros2 service call /genisom/control/release_remote std_srvs/srv/Trigger '{}'
```

必须等状态确认 `control_owner=REMOTE` 后才使用原厂遥控器。

## 主要数据

```bash
ros2 topic echo /genisom/imu
ros2 topic echo /genisom/odom
ros2 topic echo /joint_states
ros2 topic echo /genisom/battery
ros2 topic echo /genisom/current_velocity
ros2 topic echo /genisom/motor_temperatures
ros2 topic echo /genisom/faults
ros2 topic echo /diagnostics
```

所有发布项均可在 `src/genisom_l1_control/config/manager.yaml` 中配置。高层 SDK 没有传感器源时间戳和协方差，
Manager 不虚构这些数据；odom 坐标和关节正方向必须在 NUC 实机阶段核对。

## 安全提醒

- 实机动作前清空周边，保持物理急停和原厂遥控器可用。
- 高速轮足型号 `XGWHSPD` 未完成速度标定前不要提高摇杆上限。
- 软件急停不能替代物理急停。
- 官方没有解除急停 API；恢复时只释放到 REMOTE，再按原厂流程操作。
- 特殊动作 service 成功只代表命令已发送，不代表动作物理完成。
- Manager、官方示例和其他历史 SDK 节点不得同时运行。
