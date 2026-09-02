# GENISOM L1-W ROS2 Manager 架构

## 1. 互斥运行与单一 SDK 所有权

```text
Nav2 / teleop / task nodes
        | ROS topic/service
        v
二选一启动入口（禁止同时运行）
  ├── genisom_manager_node：完整控制、动作和遥测
  └── genisom_twist_node：自动 SDK、仅前后/偏航速度
        |
        v
受监督的工作子进程
  ├── ManagerNode 或 TwistNode
  └── SdkWrapper：串行化本进程唯一 ZsibotExecutor
        |
        v
官方 libzsibot.a -> UDP -> Firefly 192.168.168.168
```

两个自研目标可以同时安装，但运行时只能选择一个。官方示例仅用于 SDK 源码对照，不允许与任一 ROS2 控制
节点同时启动。

## 2. 进程监督

官方预编译 SDK 在构造期持续等待有效 model，离线时不响应 SIGINT/SIGTERM。主程序在 ROS 和 SDK 初始化前
fork 工作进程：

```text
正常 Ctrl+C
  -> 监督进程转发 SIGINT
  -> ManagerNode 析构：SDK owner 下停车、请求 REMOTE
  -> 官方 SDK 析构

构造卡死
  -> 监督进程转发 SIGINT
  -> 等待 3 秒
  -> SIGKILL 工作进程并 waitpid 回收

Twist 内部检测到遥控器接管
  -> 工作进程用 SIGUSR1 通知监督进程
  -> 监督进程向工作进程转发 SIGTERM 并启动 3 秒期限
  -> 正常关闭 SDK；若析构卡住则 SIGKILL 回收
```

工作进程设置 `PR_SET_PDEATHSIG=SIGKILL`，监督进程异常消失时不会遗留 UDP socket。

## 3. 控制权状态机

### 3.1 Manager 状态机

Manager 使用官方 `GetFunctionMode()` 作为唯一确认依据：

```text
REMOTE --request_sdk--> PENDING(SDK) --反馈 FM_SDK--> SDK
REMOTE --request_general_sdk--> PENDING(GENERAL_SDK) --> GENERAL_SDK
REMOTE --request_roamerx--> PENDING(ROAMERX) --> ROAMERX
SDK/GENERAL_SDK/ROAMERX --release_remote--> PENDING(REMOTE) --> REMOTE
```

规则：

- service 成功只表示请求已接受或反馈已在目标状态；
- `pending_control_owner` 清零且 `control_owner` 等于目标才算确认；
- 切换前关闭速度桥并清历史速度；
- 从 SDK 离开时先发送零摇杆；
- 非 REMOTE 目标超时后尝试回退 REMOTE；
- 外部人工把 SDK 改成 REMOTE 时立即关闭速度桥；
- 断线和重连不自动请求 SDK；
- 重连若遗留 `FM_SDK`，停车后释放 REMOTE。

### 3.2 Twist 状态机

```text
WAIT_CONNECTION
  -> 请求 CMD_SDK_CONTROL_RIGHT
WAIT_SDK_CONTROL
  -> 反馈 FM_SDK
WAIT_STAND
  -> 反馈 CM_STAND_UP
WAIT_MOVE_MODE
  -> 反馈 CM_MOVE_MODE
ACTIVE
  -> 接收 linear.x / angular.z
```

Twist 已确认 SDK 后，只要 FunctionMode 变为任意非 `FM_SDK` 值，就立即禁止速度发送并退出 SDK 连接。遥控器
`L2+R2+A` 组合键由底层固件识别，节点只使用 `GetFunctionMode()` 反馈，不伪造遥控器按键输入。掉线和人工接管
后都不自动重新连接或抢权。

## 4. 速度桥安全门

`/cmd_vel` 下发必须同时满足：

```text
connected
AND velocity_bridge_enabled
AND NOT estop_latched
AND NOT control_transition_pending
AND GetFunctionMode() == FM_SDK
```

任一条件不满足都不调用 `SetRemote()`。启用速度桥时先发零摇杆并清历史消息，只接受启用后的新 `/cmd_vel`。

映射严格来自官方摇杆顺序：

```text
linear.x  -> joystick[0] 前后
angular.z -> joystick[1] 旋转
linear.y  -> joystick[2] 左右
0         -> joystick[3] 头部角度
```

默认由导航器/规划器限制 ROS 物理速度，Manager 使用独立标定增益映射到归一化摇杆；也可通过参数重新启用
Manager 末端物理限幅。输出始终遵守官方 `[-1,1]` 协议边界。官方没有给出摇杆到物理速度曲线，因此标定增益
必须通过实机测量确定。

独立 Twist 模式使用相同的前进与偏航映射，但不读取 `linear.y`，并在构造 `NormalizedCommand` 时把
`lateral` 固定为 `0.0`。因此上层导航器即使错误发布横向速度，也不能绕过 SDK 下发层约束。

AUTO 下最近有效消息超过 `cmd_vel_timeout_ms` 后只发送一次零摇杆，清除 fresh 标记；下一条新消息可恢复，历史
速度不会恢复。

## 5. 姿态、档位和动作

所有非急停 `SetCmd()` 服务都要求：

- SDK 已连接；
- `GetFunctionMode() == FM_SDK`；
- 没有控制权切换；
- ESTOP 未锁存。

命令发送前统一停车并关闭速度桥。动作额外经过：

```text
GetModel()
  -> command_supported_for_model()
  -> 如需实验室模式，确认 GetMotionMode() 为 MM_REST 或 MM_RUNNING
  -> SetCmd(官方 CmdCode)
```

不自动进入实验室模式，不将锁定模式解释成阻尼模式。

## 6. ESTOP

```text
emergency_stop
  -> 关闭速度桥
  -> SDK owner 下发送零摇杆
  -> CMD_EMERGENCY_STOP
  -> estop_latched=true

clear_emergency_stop
  -> 请求/确认 REMOTE
  -> estop_latched=false
  -> 不发送站立或虚构 clear 命令
```

Manager 锁存清除不表示机器人固件已恢复，原厂恢复由操作员完成。

## 7. 遥测路径

遥测 timer 只要求 `IsConnected()`，不检查控制权：

| 官方 getter | ROS 输出 |
|---|---|
| Quaternion、BodyGyro、BodyAcc | `sensor_msgs/Imu` |
| Position、Quaternion、BodyVelocity、SpeedInfo | `nav_msgs/Odometry` |
| 12 组关节 getter | `sensor_msgs/JointState` |
| BatteryInfo | `sensor_msgs/BatteryState` |
| SpeedInfo | `geometry_msgs/TwistStamped` |
| MotorTemp[16] | `Float32MultiArray` |
| FaultInfo[] | `DiagnosticArray` |
| Model、Version | latched String topics |

所有 getter 通过同一个 `SdkWrapper::mutex_` 串行调用。Manager 当前使用单线程 ROS executor，互斥锁仍用于防止
未来 executor 调整破坏 SDK 的未声明线程安全边界。

## 8. 数据语义边界

- 四元数官方顺序 `[w,x,y,z]`，转换到 ROS 字段 `x/y/z/w`；
- 关节官方数组顺序 `[左前,右前,左后,右后]`，每腿按 `abad/hip/knee/foot` 展平；
- odom pose 使用官方世界位置和四元数，twist 使用机体速度；
- 高层 getter 没有源时间戳，ROS header 使用读取时刻；
- 官方没有给出协方差，Manager 保持默认未知值，不伪造精度；
- 官方没有给出电机温度索引名称，只发布原始 0..15；
- 不发布 odom TF，避免未经实机确认的坐标关系进入导航 TF 树。

## 9. 统一状态

`/genisom/status` 使用标准 `DiagnosticStatus`，至少包含连接、owner、control mode、speed level、motion、速度桥、
ESTOP、cmd_vel age、fault、型号、版本、pending owner、last command 和 last error。

`/diagnostics` 把 Manager 状态与每条官方 FaultInfo 合并，`/genisom/faults` 单独发布故障数组。
