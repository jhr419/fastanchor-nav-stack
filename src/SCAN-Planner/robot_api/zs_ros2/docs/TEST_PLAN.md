# GENISOM L1-W ROS2 Manager 测试计划

## 1. 状态标记

- `PASS-CODE`：代码、编译或自动测试通过；
- `PASS-HW`：实机观测通过；
- `PENDING-HW`：必须等待 NUC/机器人验证；
- `BLOCKED-UPSTREAM`：受官方 SDK 公开能力限制。

不能用 `PASS-CODE` 代替 `PASS-HW`。

## 2. 当前自动验证

| 项目 | 结果 |
|---|---|
| 官方 SDK 示例构建 | `PASS-CODE` |
| `genisom_manager_node` 构建 | `PASS-CODE` |
| `genisom_twist_node` 构建 | `PASS-CODE` |
| Twist 前后/偏航映射、横移禁用和接管策略 | `PASS-CODE` |
| 速度桥五项安全门 | `PASS-CODE` |
| XG/XGW/XGWHSPD 动作矩阵 | `PASS-CODE` |
| 实验室模式需求矩阵 | `PASS-CODE` |
| cpplint、flake8、lint_cmake、pep257、uncrustify、xmllint | `PASS-CODE` |
| cppcheck | Humble 因 2.7 性能问题默认跳过 |

当前汇总：`61 tests, 0 errors, 0 failures, 11 skipped`。

## 3. 本地无机器人测试

### T0.1 构建脚本 nounset

```bash
bash -uc './scripts/build_all.sh'
```

预期：Humble setup 不出现 `AMENT_TRACE_SETUP_FILES: unbound variable`。

### T0.2 互斥安装目标

```bash
source install/setup.bash
ros2 pkg executables genisom_l1_control
```

预期：包含 `genisom_manager_node` 和 `genisom_twist_node`，不存在旧控制节点；两个目标只能互斥运行。

### T0.3 launch 加载

```bash
ros2 launch genisom_l1_control manager.launch.py --show-args
ros2 launch genisom_l1_control twist.launch.py --show-args
```

预期：都成功显示 `config_file`，分别默认指向 `manager.yaml` 和 `twist.yaml`。

### T0.4 SDK 构造卡死退出

```bash
ros2 run genisom_l1_control genisom_manager_node --ros-args \
  -p robot_ip:=127.0.0.1 -p send_port:=18081 -p recv_port:=18080
```

出现 `model is empty` 后按一次 Ctrl+C。预期 3 秒后监督进程强制回收并退出，无 Manager 残留进程。

## 4. 实机前置条件

- 机器人处于平整封闭场地，动作测试按需要架空轮子；
- 原厂遥控器和物理急停可用；
- NUC `192.168.168.99/24` 到 Firefly `192.168.168.168` 直连可达；
- UDP `8080` 没有旧进程占用；
- Twist 模式使用旧版 highlevel 时，机器人端 `sdk_config.yaml` 的 `target_ip/target_port` 与
  `twist.yaml` 的 `local_ip/local_port` 一致；
- 不运行官方 example、旧 bridge 或其他 SDK 程序；
- 第一次验证新映射时，先在规划器中保持低速；
- 每项测试记录时间、固件版本、型号、模式前后值和现场观测。

## 5. 连接与长期遥测

### T1.1 Manager 连接

启动 Manager，读取 `/genisom/status`。

预期：`connected=true`、model=`XGWHSPD`、版本和 SN 非空；如果初始 owner 是 SDK，Manager 停车并恢复 REMOTE。

状态：`PENDING-HW`。旧工具曾确认同一设备为 `XGWHSPD / FM_REMOTE`，不等价于新 Manager 通过。

### T1.2 REMOTE 下遥测

保持 `control_owner=REMOTE`，分别检查 IMU、odom、joint_states、电池、速度、温度和 faults 的频率与数值变化。

预期：无需 SDK 控制权即可持续发布，频率接近配置值，不出现 NaN/Inf 或明显越界。

状态：`PENDING-HW`。

### T1.3 独立 Twist highlevel 启动

只启动 `twist.launch.py`，等待 `/genisom/twist/ready=true`，以低速分别发送纯 `linear.x`、纯
`angular.z`、非零 `linear.y`，以及 `linear.x + angular.z` 组合速度。

预期：状态包含 `sdk_backend=legacy_highlevel_move`；前后、横移和偏航方向正确；组合速度不会 yaw 优先抑制
`linear.x`。

状态：`PENDING-HW`。旧版 highlevel 没有新版 `FunctionMode` 反馈，Twist 不验证遥控器接管确认。

### T1.4 遥测坐标核对

静止、抬头/侧倾、手推直线、单关节可识别动作时记录 ROS 数据和实际方向。

预期：四元数顺序、加速度、角速度、odom 正方向和关节名称/符号与机器人实际一致。未确认前不得接 Nav2。

状态：`PENDING-HW`。

### T1.5 故障与电池

只使用原厂允许的可恢复测试条件观察电池错误与 FaultInfo；禁止人为制造危险硬件故障。

预期：`/genisom/faults` 与 `/diagnostics` 保留 module、submodule、error_code、level、info。

状态：`PENDING-HW`。

## 6. 控制权

### T2.1 REMOTE -> SDK

调用 request_sdk，记录 service 返回和 `/genisom/status`。

预期：先出现 pending=SDK，随后 owner=SDK、pending=NONE；速度桥仍为 false，机器人不因历史 cmd_vel 运动。

状态：`PENDING-HW`。

### T2.2 SDK -> REMOTE

SDK 下发布低速后调用 release_remote。

预期：先零摇杆停车、速度桥关闭，再请求 REMOTE；确认 owner=REMOTE 后遥控器恢复。持续发布的导航 cmd_vel 不生效。

状态：`PENDING-HW`。

### T2.3 GENERAL_SDK 与 ROAMERX

分别从 REMOTE 请求 GENERAL_SDK、ROAMERX，再释放 REMOTE。

预期：四次反馈均可确认；两种模式下速度桥不能开启，遥测持续发布。

状态：`PENDING-HW`。

### T2.4 人工强制接管

SDK 低速运行时用原厂允许方式接回 REMOTE。

预期：Manager 检测 owner 离开 SDK，关闭速度桥并清速度；不会自动请求 SDK。

状态：`PENDING-HW`。

### T2.5 切换超时

在不造成危险的条件下模拟控制权请求无反馈。

预期：`last_error` 记录目标/当前 owner；非 REMOTE 请求失败后只尝试回退 REMOTE，不循环抢 SDK。

状态：`PENDING-HW`。

## 7. 速度桥

### T3.1 五项安全门

分别在断线、bridge false、ESTOP、pending、REMOTE/GENERAL_SDK/ROAMERX 下发布 cmd_vel。

预期：均不下发运动；只有 connected + bridge true + no estop + no pending + owner SDK 时运动。

自动策略已 `PASS-CODE`，物理效果 `PENDING-HW`。

### T3.2 Twist 停发策略

Twist ready 后发布一次 `linear.x=0.01`，随后停止发布。

预期：节点只在收到消息时调用 `HighLevel::move()`；停止发布后不会在 300 ms 自动补发零速。上游发布全零
`/cmd_vel` 后机器人停车。

状态：`PENDING-HW`。

### T3.3 方向和速度标定

按 `0.01 -> 0.02 -> 0.03` 逐级测试前后、横移、旋转，并记录 highlevel 反馈速度与外部测量。

预期：建立 highlevel `move(vx, vy, yaw_rate)` 的实际速度响应、低速死区和方向符号。标定前不提高默认上限。

状态：`PENDING-HW`。

## 8. 状态、档位和动作

### T4.1 基础 ControlMode

依次验证 stand_up、sit_down、move、balance_stand、lock；每次只执行一个，并读取 `control_mode`。

预期：service 先关闭速度桥；反馈到对应官方 ControlMode。LOCK 只记录为锁定，不称阻尼。

状态：`PENDING-HW`。

### T4.2 速度档位

静止时依次请求 slow、normal、fast，读取 `speed_level`。

预期：反馈与请求一致；每次切档先关闭速度桥。快速档只验证反馈，不直接高速运动。

状态：`PENDING-HW`。

### T4.3 L1-W 支持动作

分别验证 crawl_forward、climb_high_platform；卸货下蹲先显式进入实验室模式。

预期：Manager 对 XGWHSPD 允许发送，现场按原厂条件观测动作。service success 不能单独作为动作完成依据。

状态：`PENDING-HW`。

### T4.4 轮足禁用动作

对 XGWHSPD 调用 jump、forward_jump、greet、back_flip、two_leg_stand。

预期：service `success=false`，响应包含官方型号矩阵禁止，不发送 CmdCode。

状态：策略 `PASS-CODE`，实机型号反馈路径 `PENDING-HW`。

### T4.5 双腿倒立边界

官方当前矩阵只禁止 XG 点足使用 hand_stand，未列为轮足禁用，但 L1-W 未实测。普通验收不执行该动作，只验证
service 和文档明确标记高风险待验证。

状态：`PENDING-HW`。

## 9. 急停、断线与退出

### T5.1 软件急停

低速 SDK 运行时调用 emergency_stop。

预期：速度桥关闭、停车、官方急停发出、estop_latched=true；其他状态/动作/启桥请求被拒绝。

状态：`PENDING-HW`。

### T5.2 急停恢复

调用 clear_emergency_stop。

预期：只释放并确认 REMOTE，不发送站立；Manager latch 清除后仍由原厂遥控器完成恢复。

状态：`PENDING-HW`。

### T5.3 通信断线和重连

低速 SDK 下按安全流程断开/恢复网线。

预期：断线关闭速度桥；重连不恢复旧速度、不自动请求 SDK；若固件遗留 SDK，Manager 先停车释放 REMOTE。

状态：`PENDING-HW`。

### T5.4 正常 Ctrl+C

SDK 静止和极低速两种条件下退出 Manager。

预期：SDK 下先停车并请求 REMOTE，工作进程在 3 秒内正常退出；遥控器恢复。若官方析构卡住，由监督进程回收。

状态：`PENDING-HW`。

## 10. 最终验收门槛

上线导航前至少满足：

- T1.1、T1.2、T1.3 通过；
- T2.1、T2.2、T2.4 通过；
- T3.1、T3.2、T3.3 完成；
- T4.1 的站立/移动/趴下和 T4.2 慢速通过；
- T5.1、T5.2、T5.3、T5.4 通过；
- 原厂物理急停全程可用；
- 所有结果写回本文件，保留版本和实测日期。
