# GENISOM 姿态与控制菜单使用说明书

本文说明 `scripts/sdk_cmd.sh` 的使用方法、各菜单项含义，以及 SDK 控制权、移动模式和 `/cmd_vel` 速度桥之间的
关系。适用环境为 ROS2 Humble、GENISOM L1-W 高速轮足型号 `XGWHSPD`。

## 1. 安全要求

运行脚本前必须满足：

- Manager 已连接机器人，`/genisom/status` 中 `connected=true`；
- 机器人位于平坦、空旷场地，周围没有人员和障碍物；
- 原厂遥控器在线并可接管，物理急停可用；
- 官方示例、旧 bridge 和其他直接使用 SDK 的程序没有运行；
- 键盘、导航器和其他速度源不会同时向 `/cmd_vel` 发布；
- 导航器或规划器已经配置合理的速度和加速度上限。

软件急停不能替代物理急停。高速轮足机器人在没有完成速度标定前，应从低速开始测试。

## 2. 什么是移动模式

移动模式是官方 SDK 的一种 `ControlMode`：

```text
发送命令：CMD_MOVE_MODE（0x8A）
状态反馈：CM_MOVE_MODE / MOVE_MODE
```

它表示机器人运动控制器进入可执行前后、横移和旋转输入的运动状态。移动模式本身不会产生速度，也不是导航、
自动驾驶或控制权模式。进入移动模式后，如果没有新的速度指令，机器人应保持零速度。

以下三个条件相互独立：

| 条件 | 作用 | 单独满足时能否通过 `/cmd_vel` 行走 |
|---|---|---|
| `control_owner=SDK` | Manager 获得官方 SDK 控制权 | 不能 |
| `control_mode=MOVE_MODE` | 机器人进入移动状态 | 不能 |
| `velocity_bridge_enabled=true` | 允许 Manager 转发新的 `/cmd_vel` | 不能 |

菜单标准工作流要求三者同时成立。Manager 当前对每条 `/cmd_vel` 执行的代码硬门控为：

```text
connected=true
AND control_owner=SDK
AND velocity_bridge_enabled=true
AND estop_latched=false
AND pending_control_owner=NONE
```

`control_mode=MOVE_MODE` 由菜单 `3` 在开启速度桥前单独等待确认；执行任何其他姿态命令时，Manager 又会先
关闭速度桥。因此通过菜单操作时，移动模式是开启速度桥的前置条件。不要绕过菜单，在站立、平衡站立或锁定状态
下直接调用速度桥 service。

菜单 `3` 会先发送移动模式命令、等待 `MOVE_MODE` 反馈，再开启速度桥。脚本不再提供“只进入移动模式但不开桥”
的冗余选项。

移动模式也不等于站立模式。从趴下或锁定状态恢复行走时，推荐先输入 `2`，确认机器人实际站稳，再输入 `3`。

## 3. 启动方法

终端 1 启动 Manager，并保持运行：

```bash
cd ~/zs_sdk
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch genisom_l1_control manager.launch.py
```

终端 2 启动菜单脚本：

```bash
cd ~/zs_sdk
./scripts/sdk_cmd.sh
```

输入 `1` 确认安全条件后，脚本自动执行：

```text
检查连接
→ 请求 SDK 控制权
→ 站立
→ 等待 3 秒
→ 进入移动模式
→ 开启 cmd_vel 速度桥
→ 显示交互菜单
```

初始化成功后，脚本保持运行。键盘、导航器或规划器应在另一个终端发布 `/cmd_vel`。

## 4. 菜单总览

```text
========== GENISOM 姿态与控制菜单 ==========
  0  锁存软件急停并释放到 REMOTE（不是普通姿态）
  1  趴下/坐下
  2  站立
  3  进入移动模式并开启 cmd_vel 速度桥
  4  平衡站立
  5  锁定模式
  i  显示统一状态
  h  重新显示菜单
  q  停车、关闭速度桥并切回 REMOTE
============================================
```

## 5. 菜单项详解

### 5.1 `0`：锁存软件急停

对应 Manager service 和官方命令：

```text
/genisom/emergency_stop
CMD_EMERGENCY_STOP（0x5A）
```

脚本会关闭速度桥、发送零摇杆、发送官方软件急停、锁存 Manager ESTOP，然后请求切回 REMOTE 并退出。

该操作不是普通趴下。触发后再次请求 SDK 会得到：

```text
ESTOP 已锁存，只允许释放到 REMOTE
```

再次运行脚本时会进入恢复引导。操作员必须先确认危险已经排除、遥控器可用，并按原厂流程恢复机器人；随后输入
`1`，脚本才调用 `/genisom/clear_emergency_stop` 清除 Manager 锁存。该 service 不会伪造官方不存在的底层急停
解除命令。

### 5.2 `1`：趴下/坐下

```text
Service：/genisom/mode/sit_down
官方命令：CMD_SIT_DOWN（0x6A）
预期反馈：control_mode=SIT_DOWN
```

该命令使机器人进入官方定义的趴下状态。L1-W 的具体腿部和轮部姿态以实机固件表现为准。命令执行前 Manager
停车并关闭速度桥。

恢复行走的推荐顺序：

```text
输入 2，等待实际站稳
→ 输入 3
→ 确认 velocity_bridge_enabled=true
```

### 5.3 `2`：站立

```text
Service：/genisom/mode/stand_up
官方命令：CMD_STAND_UP（0x7A）
预期反馈：control_mode=STAND_UP
```

该命令会引起明显的姿态动作。执行前必须确保机器人位于平坦地面。站立不等于可以接收速度，因为该模式命令会
关闭速度桥；需要行走时继续输入 `3`。

### 5.4 `3`：进入移动模式并开启速度桥

```text
Service：/genisom/mode/move
官方命令：CMD_MOVE_MODE（0x8A）
预期反馈：control_mode=MOVE_MODE
```

菜单执行以下组合操作：

```text
/genisom/mode/move
→ 等待 control_mode=MOVE_MODE
→ /genisom/velocity_bridge/set_enabled {data: true}
→ 等待 velocity_bridge_enabled=true
```

它不会请求新的速度，也不会恢复旧速度。速度桥开启时会清除历史 `/cmd_vel`，只接受开启后的新消息。上层停止
发布超过 `cmd_vel_timeout_ms`，默认 300 ms，Manager 会发送一次零摇杆停车。

从趴下、锁定或急停恢复时，不应把 `3` 当作站立或急停解除命令。应先完成相应恢复，再输入 `3`。

### 5.5 `4`：平衡站立

```text
Service：/genisom/mode/balance_stand
官方命令：CMD_BALANCE_STAND_MODE（0x9A）
预期反馈：control_mode=BALANCE_STAND_MODE
```

官方公开资料只给出了“平衡站立模式”的命令名称，没有公开具体平衡算法、允许输入或轮足行为。不能把它解释为
移动模式，也不能假定该状态下 `/cmd_vel` 有效。Manager 执行该命令时会停车并关闭速度桥。

### 5.6 `5`：锁定模式

```text
Service：/genisom/mode/lock
官方命令：CMD_LOCK_MODE（0x9B）
预期反馈：control_mode=LOCK_MODE
```

锁定模式严格对应官方 `LOCK_MODE`。它不是阻尼模式，官方最新高层 SDK 没有公开阻尼命令。执行锁定后速度桥关闭；
恢复移动时推荐输入 `2` 确认站立，再输入 `3`。

脚本菜单不再提供慢速、正常和快速档位选择。Manager 中对应 service 仍然存在，但日常导航速度应由上层导航器或
规划器配置，不在该简化菜单中切换固件档位。

### 5.7 `i`：显示统一状态

等价于：

```bash
ros2 topic echo --once /genisom/status
```

重点检查：

```text
connected
control_owner
control_mode
speed_level
velocity_bridge_enabled
estop_latched
pending_control_owner
fault_count / fault_summary
cmd_vel_age_ms
```

`cmd_vel_age_ms=-1` 表示当前没有仍处于看门狗有效期内的新速度，不表示通信断开。

### 5.8 `h`：重新显示菜单

只重新打印帮助信息，不发送任何机器人命令，不改变速度桥或控制权。

### 5.9 `q`：安全切回遥控器

脚本依次执行：

```text
关闭速度桥
→ 发送停车
→ 请求 REMOTE
→ 等待 control_owner=REMOTE
→ 退出脚本
```

切换完成后，即使上层仍在发布 `/cmd_vel`，Manager 也不会下发。推荐先停止导航器或键盘，再输入 `q`。
`Ctrl+C` 或终端输入结束时，脚本同样会尝试释放到 REMOTE。

## 6. 典型操作流程

### 6.1 启动导航

```text
启动 Manager
→ 运行 sdk_cmd.sh
→ 输入安全确认 1
→ 确认脚本显示“SDK 速度控制已就绪”
→ 启动导航器
→ 使用 i 持续核对状态和故障
```

如果导航器已经提前启动并持续发布 `/cmd_vel`，速度桥仍只接受开启后的新消息，但推荐先暂停导航输出再获取 SDK
控制权，以免准备完成后立即运动。

### 6.2 临时趴下后恢复导航

```text
暂停导航输出
→ 输入 1 趴下
→ 输入 2 站立
→ 等待机器人实际站稳
→ 输入 3
→ 恢复导航输出
```

### 6.3 人工接管

```text
停止导航或键盘
→ 输入 q
→ 确认 control_owner=REMOTE
→ 使用原厂遥控器
```

人工接管后 Manager 不会自动重新请求 SDK，也不会自动开启速度桥。

### 6.4 软件急停与恢复

```text
发生异常
→ 优先使用物理急停
→ 必要时输入 0 发送官方软件急停
→ Manager 锁存 ESTOP 并释放 REMOTE
→ 排除危险并按原厂流程恢复
→ 重新运行脚本
→ 按提示输入 1 清除 Manager 锁存
→ 脚本重新初始化 SDK
```

## 7. 上层 `/cmd_vel` 与限速

Manager 默认配置为：

```yaml
limit_cmd_vel_input: false
forward_joystick_per_mps: 1.0
lateral_joystick_per_mps: 2.0
yaw_joystick_per_rps: 0.5
```

默认由导航器或规划器限制 ROS 物理速度。Manager 按标定增益换算为官方归一化摇杆，并始终遵守 `[-1,1]` 协议
边界。键盘和导航器不得同时发布 `/cmd_vel`；否则两路消息会交替进入 Manager。

## 8. 状态组合与排查

| 状态 | 含义 | 处理 |
|---|---|---|
| `owner=REMOTE, bridge=false` | 遥控器控制 | 需要自动控制时重新运行脚本 |
| `owner=SDK, mode=MOVE_MODE, bridge=false` | 已进入移动模式，但速度桥关闭 | 输入 `3` |
| `owner=SDK, bridge=true, cmd_vel_age=-1` | 速度桥开启，但当前没有新速度 | 检查上层发布者 |
| `estop_latched=true` | Manager 急停锁存 | 排除危险，按脚本恢复引导处理 |
| `pending_control_owner` 不是 `NONE` | 正在等待官方控制权反馈 | 等待；超时则检查 SDK 连接 |
| `connected=false` | Manager 与 Firefly 断线 | 不发送运动命令，检查网络与唯一 SDK 进程 |

检查 `/cmd_vel` 发布和订阅：

```bash
ros2 topic info -v /cmd_vel
ros2 topic hz /cmd_vel
```

检查 Manager 状态：

```bash
ros2 topic echo --once /genisom/status
ros2 topic echo --once /genisom/faults
```

## 9. 当前验证边界

已经在 NUC 实机确认：

- Manager 能连接 `XGWHSPD` 并读取状态；
- REMOTE 到 SDK 控制权切换成功；
- 速度桥开启后键盘 `/cmd_vel` 能控制机器人；
- 菜单趴下命令能够发送并得到成功响应；
- 软件急停能够锁存，后续 SDK 请求会被拒绝。

仍需逐项记录实机结果：

- 平衡站立和锁定模式的 L1-W 实际姿态；
- 新增无物理限幅映射后的导航速度标定；
- 软件急停后的完整原厂恢复流程；
- 各姿态命令的反馈时序和物理完成时间。
