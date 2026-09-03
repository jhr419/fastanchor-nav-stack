# 智身 Twist highlevel 速度桥修改记录

## 背景

新版 `genisom_L1_sdk` 的公开 `ZsibotExecutor` 只提供 `SetCmd()` 与 `SetRemote()` 运动数据入口。
`SetRemote()` 接收的是 `[前后, 旋转, 左右, 头部角度]` 归一化摇杆值，因此旧实现需要把 ROS
`/cmd_vel` 乘标定增益后再发送。

旧版 `genisom_l1_sdk_old` 的 ZSL-1W highlevel 文档提供了 `move(float vx, float vy, float yaw_rate)`，
参数单位直接是 m/s 和 rad/s，更符合导航链路对 `/cmd_vel` 的语义。因此独立 Twist 控制节点改用旧版
highlevel 后端。

## 修改内容

### 直接速度接口

`genisom_twist_node` 现在使用：

```cpp
HighLevel::move(linear.x, linear.y, angular.z)
```

行为变化：

- 不再通过 `SetRemote()` 下发速度。
- 不再把 ROS 速度乘摇杆标定增益。
- `linear.x`、`linear.y`、`angular.z` 同时可用。
- 不再启用 yaw 优先，也不再强制 `linear.x/angular.z` 互斥。
- 可选输入限幅仍保留，但限幅单位是 m/s 和 rad/s。

### 发送策略

`/cmd_vel` 回调收到一条消息就立即调用一次 `HighLevel::move()`。

行为变化：

- 不再固定 50 Hz 持续发送当前速度。
- 不再持续发送全零速度保活。
- 不再使用 300 ms 超时清零看门狗。
- 节点退出或错误 shutdown 时仍会尝试发送一次 `move(0, 0, 0)` 做安全停车。

### 网络配置

旧版 highlevel 需要机器人端配置回传目标：

```yaml
target_ip: "控制端 IP"
target_port: 43988
```

本地 `twist.yaml` 对应参数：

```yaml
robot_ip: "192.168.168.168"
local_ip: "192.168.168.99"
local_port: 43988
```

`target_ip/target_port` 必须与 `local_ip/local_port` 一致，且 `target_ip` 与机器人网络接口在同一网段。

## 修改涉及

- `src/SCAN-Planner/robot_api/zs_ros2/third_party/genisom_l1_sdk_old`
- `src/SCAN-Planner/robot_api/zs_ros2/src/genisom_l1_control/CMakeLists.txt`
- `src/SCAN-Planner/robot_api/zs_ros2/src/genisom_l1_control/include/genisom_l1_control/sdk_wrapper.hpp`
- `src/SCAN-Planner/robot_api/zs_ros2/src/genisom_l1_control/src/sdk_wrapper.cpp`
- `src/SCAN-Planner/robot_api/zs_ros2/src/genisom_l1_control/include/genisom_l1_control/twist_node.hpp`
- `src/SCAN-Planner/robot_api/zs_ros2/src/genisom_l1_control/src/twist_node.cpp`
- `src/SCAN-Planner/robot_api/zs_ros2/src/genisom_l1_control/config/twist.yaml`
- `src/SCAN-Planner/robot_api/zs_ros2/src/genisom_l1_control/test/test_command_mapping.cpp`

## 现场确认

重新构建并加载工作区：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select genisom_l1_control
source install/setup.bash
ros2 launch genisom_l1_control twist.launch.py
```

观察 Twist 状态：

```bash
ros2 topic echo /genisom/twist/status
```

预期行为：

- 状态中 `sdk_backend=legacy_highlevel_move`。
- 状态中 `cmd_vel_send_policy=on_message`。
- 状态中 `cmd_vel_timeout_ms=disabled`。
- 只发送 `linear.x` 时按前后速度运动。
- 发送非零 `linear.y` 时按侧向速度运动。
- 同时发送 `linear.x` 和 `angular.z` 时两者同时传入 `move()`，不会 yaw 优先抑制 `linear.x`。
- 停止发布 `/cmd_vel` 后节点不会自行补发零速；如需停车，上游必须发布零速度。

## 注意

旧版 highlevel 接口没有新版 `FunctionMode` 反馈，Twist 节点无法像新版 `ZsibotExecutor` 一样确认遥控器是否接管。
现场需要保留物理急停和原厂遥控器，首次验证请使用低速并确保周围空旷。
