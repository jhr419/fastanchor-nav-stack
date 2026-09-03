# GENISOM L1-W ROS2 Manager 用户手册

## 1. 功能定位

`GENISOM L1-W ROS2 Manager` 是机器人官方 SDK 的完整 ROS2 入口。项目另提供独立 Twist 控制模式；
Twist 参考旧版 ZSL-1W highlevel SDK，直接把 `/cmd_vel` 传给 `HighLevel::move(vx, vy, yaw_rate)`。

- `REMOTE / SDK / GENERAL_SDK / ROAMERX` 控制权请求与反馈确认；
- 站立、趴下、移动、平衡站立、锁定和官方软急停；
- 慢速、正常、快速档位切换；
- 官方特殊动作下发和基于 `GetModel()` 的型号屏蔽；
- Manager 的 `/cmd_vel` 到 `SetRemote()` 安全速度桥；
- Twist 的 `/cmd_vel` 到 `HighLevel::move()` 直接速度桥；
- IMU、odom、关节、电池、故障、温度、型号和版本等长期发布；
- `/genisom/status` 和 `/diagnostics` 统一状态输出；
- 官方 SDK 构造卡死时的进程级退出保护。

Nav2、键盘遥控和任务程序只能通过当前选定的 Manager 或 Twist ROS 接口控制机器人。现场运行时仍建议
Manager、Twist 和官方示例三者之间互斥，避免多进程同时给运动控制发速度。

## 2. 官方依据与验证边界

实现基于官方仓库 `https://github.com/zsibot/genisom_L1_sdk` 的 `main` 分支，当前提交：

```text
f7ccbf393e96f1205af8cce8070c5886f9641428
```

已对照阅读：

- `third_party/genisom_L1_sdk/README.md`
- `third_party/genisom_L1_sdk/CHANGELOG.md`
- `third_party/genisom_L1_sdk/docs/zh/api.md`
- `third_party/genisom_L1_sdk/docs/zh/api_lowlevel.md`
- `third_party/genisom_L1_sdk/docs/protocol/protocol.md`
- `third_party/genisom_L1_sdk/include/zsibot_sdk/zsibot_api.h`
- `third_party/genisom_L1_sdk/include/zsibot_sdk/zsibot_define.h`
- `third_party/genisom_L1_sdk/example/sdk_control.cpp`
- `third_party/genisom_L1_sdk/example/remote_control.cpp`
- `third_party/genisom_l1_sdk_old/docs/api_zsl-1w.md`
- `third_party/genisom_l1_sdk_old/include/zsl-1w/highlevel.h`

当前验证状态：

| 范围 | 状态 |
|---|---|
| Manager、SDK wrapper、launch、配置代码 | 已完成 |
| 独立 Twist 节点、启动文件、配置和接管状态机 | 已完成 |
| x86_64 / ROS2 Humble 编译 | 已通过 |
| 命令映射、速度桥授权、型号动作矩阵单测 | 已通过 |
| lint 与包测试 | 已通过 |
| 官方 SDK 最新提交核对 | 已通过，远端与本地一致 |
| 旧版工具连接 Firefly、识别 `XGWHSPD`、读取 `FM_REMOTE` | 已实机确认 |
| 新 Manager 连接、所有 topic 数据正确性 | 待移植 NUC 后实机验证 |
| 控制权四模式往返切换 | 待实机验证 |
| 姿态、档位、特殊动作、急停 | 待实机验证 |
| IMU/odom/关节坐标方向与机器人 URDF 一致性 | 待实机验证 |
| `/cmd_vel` 物理速度比例与方向 | 待实机标定 |
| `L2+R2+A` 两秒接管、节点退出和端口释放 | 待 NUC 实机验证 |

“已编译”不等于“机器人动作已验证”。未完成实机测试前，必须在封闭场地、低速和物理急停可用的条件下逐项验证。

## 3. 目录结构

```text
.
├── README.md
├── docs
│   ├── CONTROL_ARCHITECTURE.md
│   ├── SDK_CMD_MENU_GUIDE.md
│   ├── SDK_ANALYSIS.md
│   ├── TEST_PLAN.md
│   └── USER_GUIDE.md
├── scripts
│   ├── build_all.sh
│   └── sdk_cmd.sh
├── src/genisom_l1_control
│   ├── config
│   │   ├── manager.yaml
│   │   └── twist.yaml
│   ├── include/genisom_l1_control
│   │   ├── manager_node.hpp
│   │   ├── process_supervisor.hpp
│   │   ├── sdk_wrapper.hpp
│   │   └── twist_node.hpp
│   ├── launch
│   │   ├── manager.launch.py
│   │   ├── twist.launch.py
│   │   └── control.launch.py
│   ├── src
│   │   ├── main.cpp
│   │   ├── manager_node.cpp
│   │   ├── process_supervisor.cpp
│   │   ├── sdk_wrapper.cpp
│   │   ├── twist_main.cpp
│   │   └── twist_node.cpp
│   └── test/test_command_mapping.cpp
└── third_party/genisom_L1_sdk
```

`control.launch.py` 仅作为旧启动命令的兼容入口，内部启动的仍是唯一 `genisom_manager_node`。

## 4. 依赖与编译

环境：Ubuntu 22.04、ROS2 Humble、GCC 11 或兼容 C++20 编译器，架构为 x86_64 或 aarch64。

```bash
cd zs_sdk
source /opt/ros/humble/setup.bash
./scripts/build_all.sh
source install/setup.bash
```

脚本先构建官方示例，再构建 Manager。它会清理本包的旧安装目录，避免历史
`genisom_control_node` 或 `control_right_test` 残留并绕过 Manager。

运行测试：

```bash
colcon test --packages-select genisom_l1_control
colcon test-result --verbose
```

检查部署目标：

```bash
ros2 pkg executables genisom_l1_control
```

预期包含：

```text
genisom_l1_control genisom_manager_node
genisom_l1_control genisom_twist_node
```

## 5. 启动 Manager

先确认 Firefly 在线且没有其他 SDK 程序：

```bash
ping -c 3 192.168.168.168
pgrep -af '[s]dk_control|[r]emote_control|[g]enisom_manager_node'
ss -lunp | grep ':8080'
```

启动：

```bash
ros2 launch genisom_l1_control manager.launch.py
```

指定另一份参数文件：

```bash
ros2 launch genisom_l1_control manager.launch.py \
  config_file:=src/genisom_l1_control/config/manager.yaml
```

Manager 启动时：

- 不自动请求 SDK 控制权；
- 速度桥默认关闭；
- 若首次/重连反馈仍是 `FM_SDK`，先停车并请求恢复 REMOTE；
- 处于 REMOTE、GENERAL_SDK 或 ROAMERX 时仍持续发布遥测；
- 断线后关闭速度桥，重连后不会自动抢回 SDK 控制权。

### 5.1 独立 Twist 控制模式

仅需要启动后直接接受导航或键盘 `/cmd_vel` 时，可不启动 Manager，改用：

```bash
ros2 launch genisom_l1_control twist.launch.py
```

它会依次完成：

1. 等待官方 SDK 连接；
2. 发送 `CMD_SDK_CONTROL_RIGHT` 并等待 `FunctionMode=FM_SDK`；
3. 发送站立命令并等待 `ControlMode=CM_STAND_UP`；
4. 发送移动模式命令并等待 `ControlMode=CM_MOVE_MODE`；
5. 发布 `/genisom/twist/ready=true`，开始接受新的 `/cmd_vel`。

查看状态：

```bash
ros2 topic echo --once /genisom/twist/ready
ros2 topic echo /genisom/twist/status
```

发送速度：

```bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.02, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.1}}'
```

该节点使用旧版 ZSL-1W highlevel 接口。收到一条 `/cmd_vel` 后立即调用一次
`HighLevel::move(linear.x, linear.y, angular.z)`；不再通过 `SetRemote()`，不再乘摇杆增益，不再做 yaw
优先或 `linear.x/angular.z` 互斥，也不再启用 300 ms 超时清零看门狗。停止发布 `/cmd_vel` 时，节点不会自行
补发零速；上游需要停车时必须发布一条全零速度。

旧版 highlevel 接口没有新版 `FunctionMode` 反馈，Twist 节点不能确认遥控器接管状态。现场仍需保留原厂遥控器
和物理急停；如需重新进入 Twist，必须在人员确认安全后重新执行 `twist.launch.py`。

以下程序不得与 Twist 节点同时运行：

```text
manager.launch.py
control.launch.py
官方 sdk_control / remote_control 示例
其他占用 highlevel 本地端口的历史 SDK 节点
```

Twist 模式只负责速度控制和自身状态，不发布 Manager 的 IMU、odom、关节、电池和故障 topic。需要完整遥测、
动作 service 或统一 diagnostics 时，应使用 Manager 模式。

Twist 参数位于 `src/genisom_l1_control/config/twist.yaml`：

| Parameter | 默认值 | 用途 |
|---|---:|---|
| `robot_ip` | `192.168.168.168` | Firefly 地址 |
| `local_ip` | `192.168.168.99` | 机器人端 `sdk_config.yaml` 的 `target_ip` |
| `local_port` | `43988` | 机器人端 `sdk_config.yaml` 的 `target_port` |
| `cmd_vel_topic` | `/cmd_vel` | 速度输入 topic |
| `connection_timeout_ms` | `10000` | 首次连接等待时间 |
| `mode_command_timeout_ms` | `8000` | 站立确认时间 |
| `command_retry_ms` | `500` | 站立命令重试周期 |
| `status_rate_hz` | `5.0` | Twist 状态发布频率 |
| `auto_stand` | `true` | highlevel 连接后自动站立 |
| `limit_cmd_vel_input` | `false` | 是否启用末端物理速度限幅 |
| `max_linear_x` / `max_linear_y` / `max_angular_z` | `3.7` / `1.0` / `3.0` | 启用限幅后的上限 |
| `linear_deadband` / `angular_deadband` | `0.0` / `0.0` | 物理速度死区 |

`local_ip/local_port` 必须与机器人端 `/opt/export/config/sdk_config.yaml` 的 `target_ip/target_port` 一致。

## 6. 统一状态

```bash
ros2 topic echo /genisom/status
```

消息类型为 `diagnostic_msgs/msg/DiagnosticStatus`，主要 `values`：

| key | 含义 |
|---|---|
| `connected` | SDK 心跳是否有效 |
| `control_owner` | `REMOTE / SDK / GENERAL_SDK / ROAMERX / TRACE / NULL` |
| `control_mode` | 站立、趴下、移动、平衡站立、锁定、急停等官方反馈 |
| `speed_level` | `SLOW / NORMAL / FAST / NULL` |
| `motion_mode` | 特殊动作执行状态 |
| `motion_type` | 官方可反馈的动作类型 |
| `velocity_bridge_enabled` | `/cmd_vel` 桥是否开启 |
| `cmd_vel_input_limit_enabled` | Manager 是否启用 ROS 物理速度限幅 |
| `forward_joystick_per_mps` 等 | 当前物理速度到归一化摇杆的标定增益 |
| `estop_latched` | Manager ESTOP 是否锁存 |
| `cmd_vel_age_ms` | 最近有效速度消息年龄；无有效消息为 `-1` |
| `fault_count` | 当前官方故障条数 |
| `fault_summary` | 第一条故障摘要，无故障为 `none` |
| `pending_control_owner` | 正在等待确认的控制权目标 |
| `model` | `XG / XGW / XGWHSPD` |
| `serial_number`、`device_name`、`wifi_ssid` | 官方设备基本信息 |
| `device_temperature_c`、`battery_power_percent` | 设备温度和电量摘要 |
| `forward_speed_mps`、`lateral_speed_mps`、`yaw_rate_rps` | 官方当前速度摘要 |
| `mc_version`、`dog_task_version` | 官方版本信息 |
| `last_command`、`last_error` | 最近 Manager 操作和错误 |

快速查看关键字段：

```bash
ros2 topic echo --once /genisom/status
ros2 topic echo --once /genisom/connection
ros2 topic echo --once /genisom/model
ros2 topic echo --once /genisom/version
```

## 7. Service 总表

### 7.1 控制权

| Service | 类型 | 用途 |
|---|---|---|
| `/genisom/control/request_sdk` | `std_srvs/srv/Trigger` | 请求 `FM_SDK` |
| `/genisom/control/request_general_sdk` | `Trigger` | 请求 `FM_GENERAL_SDK` |
| `/genisom/control/request_roamerx` | `Trigger` | 请求 `FM_ROAMER` |
| `/genisom/control/release_remote` | `Trigger` | 停车、关闭速度桥并请求 `FM_REMOTE` |
| `/genisom/take_auto_control` | `Trigger` | 兼容旧脚本，等价于 request_sdk |
| `/genisom/release_to_remote` | `Trigger` | 兼容旧脚本，等价于 release_remote |

Service 返回成功表示命令已发送或反馈已经是目标模式。最终结果必须看 `/genisom/status` 的
`control_owner` 和 `pending_control_owner`。

### 7.2 速度桥与急停

| Service | 类型 | 用途 |
|---|---|---|
| `/genisom/velocity_bridge/set_enabled` | `std_srvs/srv/SetBool` | 开启或关闭 `/cmd_vel` 下发 |
| `/genisom/emergency_stop` | `Trigger` | 关闭速度桥、停车并发送官方软急停 |
| `/genisom/clear_emergency_stop` | `Trigger` | 只请求/确认 REMOTE 并清 Manager 锁存 |

官方 SDK 没有独立的解除急停 API。`clear_emergency_stop` 不会把 `CMD_STAND_UP` 冒充解除急停；机器人恢复必须按
原厂遥控器流程执行。

### 7.3 运动状态

| Service | 官方命令 |
|---|---|
| `/genisom/mode/stand_up` | `CMD_STAND_UP` |
| `/genisom/mode/sit_down` | `CMD_SIT_DOWN` |
| `/genisom/mode/move` | `CMD_MOVE_MODE` |
| `/genisom/mode/balance_stand` | `CMD_BALANCE_STAND_MODE` |
| `/genisom/mode/lock` | `CMD_LOCK_MODE` |
| `/genisom/mode/enter_lab` | `CMD_ENTER_LAB_MODE` |
| `/genisom/mode/exit_lab` | `CMD_EXIT_LAB_MODE` |
| `/genisom/mode/enter_entertainment` | `CMD_ENTER_ENTERTAINMENT_MODE` |
| `/genisom/mode/exit_entertainment` | `CMD_EXIT_ENTERTAINMENT_MODE` |

锁定模式严格按官方名称 `LOCK_MODE` 处理。最新高层 SDK 中不存在 damping/阻尼枚举或命令，因此 Manager 不提供
阻尼服务。

### 7.4 速度档位

| Service | 官方命令 |
|---|---|
| `/genisom/speed/slow` | `CMD_SLOW_SPEED` |
| `/genisom/speed/normal` | `CMD_NORMAL_SPEED` |
| `/genisom/speed/fast` | `CMD_FAST_SPEED` |

### 7.5 特殊动作

| Service | 官方命令 |
|---|---|
| `/genisom/action/crawl_forward` | `CMD_CRAWL_FORWARD` |
| `/genisom/action/climb_high_platform` | `CMD_CLIMBING_HIGH_PLATFORM` |
| `/genisom/action/unload_squat` | `CMD_UNLOAD_SQUAT` |
| `/genisom/action/jump` | `CMD_JUMP` |
| `/genisom/action/forward_jump` | `CMD_FORWARD_JUMP` |
| `/genisom/action/back_flip` | `CMD_BACK_FLIP` |
| `/genisom/action/greet` | `CMD_GREET` |
| `/genisom/action/two_leg_stand` | `CMD_TWO_LEG_STAND` |
| `/genisom/action/hand_stand` | `CMD_HAND_STAND` |

姿态、档位和动作服务都要求当前反馈为 `FM_SDK`，并会先停车、关闭速度桥。命令完成后如需继续导航，必须重新
调用速度桥开启服务。

## 8. Topic 总表

| Topic | 类型 | 说明 |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 上层速度输入，仅通过安全门控后下发 |
| `/genisom/status` | `diagnostic_msgs/msg/DiagnosticStatus` | Manager 统一状态 |
| `/genisom/connection` | `std_msgs/msg/Bool` | 连接状态，latched |
| `/genisom/model` | `std_msgs/msg/String` | 官方型号，latched |
| `/genisom/version` | `std_msgs/msg/String` | MC 与 dog_task 版本，latched |
| `/genisom/imu` | `sensor_msgs/msg/Imu` | 官方四元数、角速度和机体加速度 |
| `/genisom/odom` | `nav_msgs/msg/Odometry` | 官方位置、姿态和机体速度 |
| `/joint_states` | `sensor_msgs/msg/JointState` | 4 腿 x 4 关节的位置、速度和力矩 |
| `/genisom/battery` | `sensor_msgs/msg/BatteryState` | 电压、电流、温度、电量和错误状态 |
| `/genisom/current_velocity` | `geometry_msgs/msg/TwistStamped` | 官方 `GetSpeed()` 反馈 |
| `/genisom/motor_temperatures` | `std_msgs/msg/Float32MultiArray` | 官方 16 路原始温度数组 |
| `/genisom/faults` | `diagnostic_msgs/msg/DiagnosticArray` | 官方故障列表 |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | Manager 状态和故障汇总 |

高层 SDK getter 不提供每包原始时间戳，因此 ROS header 使用 Manager 读取/发布时间。官方高层文档没有给出 16 路
电机温度数组的名称映射，因此该 topic 只发布 `raw_motor_index`，不猜测电机名称。

## 9. Parameter 总表

| Parameter | 默认值 | 用途 |
|---|---:|---|
| `robot_ip` | `192.168.168.168` | 当前 Firefly 地址 |
| `send_port` | `8081` | 机器人接收端口 |
| `recv_port` | `8080` | Manager 本地接收端口 |
| `cmd_vel_topic` | `/cmd_vel` | 速度输入 topic |
| `joint_states_topic` | `/joint_states` | 关节输出 topic |
| `cmd_vel_timeout_ms` | `300` | 速度看门狗超时 |
| `mode_switch_timeout_ms` | `2000` | 控制权反馈确认超时 |
| `telemetry_rate_hz` | `20.0` | 遥测读取与发布频率 |
| `status_rate_hz` | `5.0` | 状态和 diagnostics 频率 |
| `imu_frame_id` | `imu_link` | IMU frame |
| `odom_frame_id` | `odom` | odom 父 frame |
| `base_frame_id` | `base_link` | odom 子 frame、速度 frame |
| `publish_imu` | `true` | 是否发布 IMU |
| `publish_odom` | `true` | 是否发布 odom |
| `publish_joint_states` | `true` | 是否发布关节 |
| `publish_battery` | `true` | 是否发布电池 |
| `publish_velocity` | `true` | 是否发布当前速度 |
| `publish_motor_temperatures` | `true` | 是否发布电机温度 |
| `publish_diagnostics` | `true` | 是否发布 `/diagnostics` |
| `limit_cmd_vel_input` | `false` | 是否在 Manager 中限制 ROS 物理速度 |
| `max_linear_x` | `0.10` | 启用输入限幅时的前后上限，m/s |
| `max_linear_y` | `0.05` | 启用输入限幅时的横移上限，m/s |
| `max_angular_z` | `0.20` | 启用输入限幅时的旋转上限，rad/s |
| `forward_joystick_per_mps` | `1.0` | 前后速度到摇杆值的标定增益 |
| `lateral_joystick_per_mps` | `2.0` | 横移速度到摇杆值的标定增益 |
| `yaw_joystick_per_rps` | `0.5` | 旋转速度到摇杆值的标定增益 |
| `max_forward_joystick` | `1.0` | 官方前后归一化摇杆协议边界 |
| `max_lateral_joystick` | `1.0` | 官方横移归一化摇杆协议边界 |
| `max_yaw_joystick` | `1.0` | 官方旋转归一化摇杆协议边界 |

默认由导航器或规划器负责物理速度限制。Manager 使用
`joystick = cmd_vel * joystick_per_*` 完成单位映射，并始终把结果限制在官方 SDK 的 `[-1,1]` 协议范围。
如需恢复 Manager 末端物理限幅，将 `limit_cmd_vel_input` 设为 `true`。参数在启动时读取，修改 YAML 后重启
Manager 才生效。

## 10. 进入 SDK 控制并发送速度

数字菜单的逐项说明、移动模式含义和典型操作流程见 `docs/SDK_CMD_MENU_GUIDE.md`。

Manager 已启动后，可以运行以下脚本完成基础速度控制初始化：

```bash
./scripts/sdk_cmd.sh
```

脚本按顺序检查连接、请求 SDK、站立、进入移动模式，并最后开启速度桥。默认要求操作员输入
`1`；明确用于自动化时可执行 `./scripts/sdk_cmd.sh --yes`。任何步骤失败或脚本被中断时，脚本会尝试关闭
速度桥并释放回 REMOTE。初始化成功后脚本显示以下交互菜单并保持运行，上层可在另一个终端发布 `/cmd_vel`：

| 输入 | 操作 |
|---|---|
| `0` | 锁存软件急停并释放到 REMOTE，不是普通姿态 |
| `1` | 趴下/坐下 |
| `2` | 站立 |
| `3` | 进入移动模式并开启 `/cmd_vel` 速度桥 |
| `4` | 平衡站立 |
| `5` | 锁定模式 |
| `i` | 显示 `/genisom/status` |
| `h` | 重新显示菜单 |
| `q` | 停车、关闭速度桥并切回 REMOTE |

姿态命令都会关闭速度桥。执行 `1`、`2`、`4` 或 `5` 后如需继续接收 `/cmd_vel`，必须输入 `3`。脚本菜单
不区分速度档位，速度由上层导航器或规划器管理。按 `Ctrl+C` 或终端输入结束时，脚本也会尝试安全释放到
REMOTE。

输入 `0` 后 Manager ESTOP 会保持锁存，后续 SDK 请求会被拒绝。再次运行脚本时会先检测该状态，要求操作员确认
危险已经排除、遥控器可用并按原厂流程恢复机器人；再次输入 `1` 后，脚本才调用
`/genisom/clear_emergency_stop` 清除 Manager 锁存、确认 REMOTE，然后重新进入 SDK 初始化。该流程不会伪造官方
不存在的底层急停解除命令。

以下内容为等价的手工操作流程。

请求 SDK：

```bash
ros2 service call /genisom/control/request_sdk std_srvs/srv/Trigger '{}'
ros2 topic echo /genisom/status
```

确认 `control_owner=SDK`、`pending_control_owner=NONE` 后，根据机器人现场状态发送站立或移动命令：

```bash
ros2 service call /genisom/mode/stand_up std_srvs/srv/Trigger '{}'
ros2 service call /genisom/mode/move std_srvs/srv/Trigger '{}'
ros2 service call /genisom/speed/slow std_srvs/srv/Trigger '{}'
```

每个命令都会关闭速度桥。状态稳定后显式开启：

```bash
ros2 service call /genisom/velocity_bridge/set_enabled std_srvs/srv/SetBool \
  '{data: true}'
```

低速短消息测试：

```bash
ros2 topic pub --rate 10 --times 3 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.01, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

停止发布超过 300 ms 后 Manager 发送零摇杆。速度桥保持开启，但只接受超时后的新消息，不恢复历史速度。

关闭速度桥：

```bash
ros2 service call /genisom/velocity_bridge/set_enabled std_srvs/srv/SetBool \
  '{data: false}'
```

## 11. 切回原厂遥控器与人工接管

推荐顺序：

1. 暂停导航器输出；
2. 调用 release_remote；
3. Manager 自动关闭速度桥并停车；
4. 等待 `/genisom/status` 确认 `control_owner=REMOTE`；
5. 再操作原厂遥控器。

```bash
ros2 service call /genisom/control/release_remote std_srvs/srv/Trigger '{}'
ros2 topic echo /genisom/status
```

即使导航器仍在发布 `/cmd_vel`，REMOTE 下这些消息也会被丢弃。人工接管后 Manager 不会自动请求 SDK，也不会
自动重新开启速度桥。恢复自动导航必须重新执行 request_sdk、确认 owner、设置所需模式，然后显式开启速度桥。

如果原厂方式直接把 FunctionMode 从 SDK 改为 REMOTE，Manager 检测到反馈变化后同样会关闭速度桥并清除历史速度。

## 12. GENERAL_SDK 与 ROAMERX

```bash
ros2 service call /genisom/control/request_general_sdk std_srvs/srv/Trigger '{}'
ros2 service call /genisom/control/request_roamerx std_srvs/srv/Trigger '{}'
```

切换前 Manager 会关闭速度桥；即使进入 `GENERAL_SDK` 或 `ROAMERX`，`/cmd_vel` 也不会下发。当前 Manager 的
速度桥只允许官方 `FM_SDK`，避免把不同控制者的语义混在一起。遥测在这些模式下继续发布。

## 13. 状态与动作示例

所有命令先确认 `control_owner=SDK`。

```bash
ros2 service call /genisom/mode/stand_up std_srvs/srv/Trigger '{}'
ros2 service call /genisom/mode/sit_down std_srvs/srv/Trigger '{}'
ros2 service call /genisom/mode/balance_stand std_srvs/srv/Trigger '{}'
ros2 service call /genisom/mode/lock std_srvs/srv/Trigger '{}'

ros2 service call /genisom/action/crawl_forward std_srvs/srv/Trigger '{}'
ros2 service call /genisom/action/climb_high_platform std_srvs/srv/Trigger '{}'
```

卸货下蹲按官方说明需要实验室模式：

```bash
ros2 service call /genisom/mode/enter_lab std_srvs/srv/Trigger '{}'
ros2 topic echo --once /genisom/status
ros2 service call /genisom/action/unload_squat std_srvs/srv/Trigger '{}'
ros2 service call /genisom/mode/exit_lab std_srvs/srv/Trigger '{}'
```

Manager 只有在 `motion_mode=REST` 或 `RUNNING` 已确认时才允许需要实验室模式的动作；`FORBID` 或 `NULL`
都会拒绝。官方 `MotionType` 没有为匍匐前进、爬高台和双腿倒立定义独立反馈枚举，因此 service 成功只表示
命令已发送，不能视为动作物理完成。

## 14. L1-W 型号限制

实机已读取型号 `XGWHSPD`。Manager 每次执行动作前调用 `GetModel()`，按官方矩阵判定：

| 动作 | XG 点足 | XGW/XGWHSPD 轮足 |
|---|---|---|
| 匍匐前进 | 拒绝 | 允许下发，待实机验证 |
| 爬高台 | 拒绝 | 允许下发，待实机验证 |
| 双腿倒立 | 拒绝 | 官方未列为轮足禁用，允许下发但待实机验证 |
| 卸货下蹲 | 允许，需实验室模式 | 允许，需实验室模式，待实机验证 |
| 跳跃/前跳/招手/后空翻/双腿站立 | 允许，部分需实验室模式 | 自动拒绝 |

这张表只复述当前官方仓库。对于“允许下发但未实机验证”的高风险动作，不应在普通部署中尝试。

## 15. 查看遥测

```bash
ros2 topic hz /genisom/imu
ros2 topic echo /genisom/imu
ros2 topic echo /genisom/odom
ros2 topic echo /joint_states
ros2 topic echo /genisom/battery
ros2 topic echo /genisom/current_velocity
ros2 topic echo /genisom/motor_temperatures
ros2 topic echo /genisom/faults
ros2 topic echo /diagnostics
```

关节 getter 的官方数组顺序是 `[左前, 右前, 左后, 右后]`，每腿关节为 `abad/hip/knee/foot`。Manager 转为
16 个名称，但关节正方向、零点和 URDF 轴仍必须实机/模型核对。Manager 不发布 odom TF；在确认坐标系与定位
系统关系前，不要直接把官方 odom 当作导航唯一定位源。

## 16. 典型导航流程

```text
启动机器人和原厂遥控器
→ 启动 Manager，确认 connected=true、owner=REMOTE
→ 请求 SDK，等待 owner=SDK
→ 设置站立/移动状态
→ 启动定位与导航，但先保持速度输出暂停
→ 开启 velocity bridge
→ 恢复导航速度输出
→ 持续监控 /genisom/status 和 /diagnostics
```

停止导航：

```text
暂停导航输出
→ 关闭 velocity bridge（停车）
→ release_remote
→ 确认 owner=REMOTE
```

## 17. 急停流程

软件急停：

```bash
ros2 service call /genisom/emergency_stop std_srvs/srv/Trigger '{}'
```

执行后速度桥关闭、零摇杆发送、`CMD_EMERGENCY_STOP` 下发，Manager ESTOP 锁存。物理危险时优先使用原厂物理
急停，ROS service 不能替代硬件安全链路。

恢复：

```bash
ros2 service call /genisom/clear_emergency_stop std_srvs/srv/Trigger '{}'
```

该服务仅恢复/确认 REMOTE 并清 Manager 锁存。随后由操作员使用原厂遥控器按原厂流程恢复机器人。恢复后
Manager 不自动请求 SDK。

## 18. 断线与异常处理

断线时 Manager：

- 立即关闭速度桥并清除历史 `/cmd_vel`；
- 不再调用 `SetRemote()`；
- 状态变为 `connected=false`；
- 重连后不会自动请求 SDK；
- 如果重连反馈遗留为 SDK，会先停车并请求 REMOTE。

控制权请求超过 `mode_switch_timeout_ms` 未确认时，状态写入 `last_error`。非 REMOTE 目标超时后 Manager 会尝试
回退到 REMOTE，但不会循环抢 SDK。

官方 SDK 构造期间若收不到有效 model，会反复输出：

```text
model is empty, please wait..
```

按一次 Ctrl+C。监督进程先给工作进程 3 秒正常退出时间，仍卡死则强制回收，不需要手工查杀 PID。

常用排查：

```bash
ip route get 192.168.168.168
ping -c 3 192.168.168.168
ss -lunp | grep ':8080'
ros2 topic echo --once /genisom/status
ros2 service list | grep genisom
ros2 topic list | grep genisom
```

## 19. 当前不实现的功能

- 阻尼模式：官方最新高层 SDK 没有公开 damping CmdCode 或 ControlMode；锁定模式不等于阻尼模式。
- 自动解除官方软急停：官方没有 ClearEmergencyStop API。
- GENERAL_SDK/ROAMERX 下的 `/cmd_vel`：语义与权限未由官方文档定义，Manager 明确禁止。
- 自动抢回 SDK：人工接管或断线重连后必须显式调用 service。
- 自动进入实验室模式：高风险动作必须由用户显式进入，Manager 不隐式切换。
- 猜测电机温度名称、传感器源时间戳、未公开协方差或 odom TF：官方高层 API 未提供足够依据。
