# 智身 Twist 心跳与运动互斥修改记录

## 问题现象

`genisom_twist_node` 进入 SDK 控制并正常运行后，在上游停止发布 `cmd_vel` 时先触发 300 ms 零速看门狗，约 5 秒后又被判定为 SDK 连接丢失并退出。机器狗执行同时包含 `linear.x` 和 `angular.z` 的命令时会走弧线，现场观感类似横向偏移。

## 原因分析

官方 SDK 使用 50 ms 心跳和 5 秒连接超时。原节点只在收到新 `cmd_vel` 时调用 `SetRemote()`，看门狗超时后只发送一次零速，之后没有周期控制包。连接判定变为 `false` 后，节点又会在第一次采样时立即退出，没有给瞬时丢包或零速探测留出恢复时间。

官方 `SetRemote()` 摇杆顺序为 `[前后, 旋转, 左右, 头部角度]`。现有第三个左右轴已经固定为 `0`，没有直接下发 `linear.y`。现场横向观感主要来自前进和旋转同时执行形成的弧线，因此需要把两种运动改为互斥。

## 修改内容

### 固定频率发送与连接宽限期

修改 `src/SCAN-Planner/robot_api/zs_ros2/src/genisom_l1_control/src/twist_node.cpp`：

- ACTIVE 状态下每 20 ms 发送一次当前归一化命令。
- 超过 `cmd_vel_timeout_ms` 后将保存的命令清零，并继续周期发送零速。
- SDK 心跳中断时进入 `disconnect_grace_ms` 宽限期，清除旧速度并持续发送零速探测。
- 心跳恢复后等待新的 `cmd_vel`，不恢复断线前的运动命令。
- 只有连接持续丢失超过宽限期才退出。
- 遥控器接管导致 `FunctionMode` 离开 `FM_SDK` 时仍然退出，不自动重新抢权。

`src/SCAN-Planner/robot_api/zs_ros2/src/genisom_l1_control/config/twist.yaml` 新增：

```yaml
disconnect_grace_ms: 10000
```

### 平移与转向互斥

智身 Twist 末端和 SCAN 闭环控制器均使用 yaw 优先策略：

- `abs(angular.z) > angular_deadband` 时，强制 `linear.x=0`，只原地转向。
- `abs(angular.z) <= angular_deadband` 时，强制 `angular.z=0`，允许前进或后退。
- `linear.y` 始终为 `0`。
- SCAN 的 `forward_only` 改为 `false`，允许负的 `linear.x`。

默认参数：

```yaml
forward_only: false
exclusive_translation_rotation: true
linear_deadband: 0.01
angular_deadband: 0.05
```

修改涉及：

- `src/SCAN-Planner/planner/plan_manage/include/plan_manage/motion_constraints.hpp`
- `src/SCAN-Planner/planner/plan_manage/src/closed_loop_controller.cpp`
- `src/SCAN-Planner/planner/plan_manage/src/go2_kinematic_sim.cpp`
- `src/SCAN-Planner/planner/plan_manage/config/controllers.yaml`
- `src/SCAN-Planner/robot_api/zs_ros2/src/genisom_l1_control/include/genisom_l1_control/sdk_wrapper.hpp`
- `src/SCAN-Planner/robot_api/zs_ros2/src/genisom_l1_control/src/sdk_wrapper.cpp`
- `src/SCAN-Planner/robot_api/zs_ros2/src/genisom_l1_control/include/genisom_l1_control/twist_node.hpp`
- `src/SCAN-Planner/robot_api/zs_ros2/src/genisom_l1_control/src/twist_node.cpp`
- `src/SCAN-Planner/robot_api/zs_ros2/src/genisom_l1_control/config/twist.yaml`

## 现场确认

重新构建并加载工作区：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select genisom_l1_control scan_planner
source install/setup.bash
ros2 launch genisom_l1_control twist.launch.py
```

观察 SDK 末端状态：

```bash
ros2 topic echo /genisom/twist/status
```

预期行为：

- 没有 `cmd_vel` 时保持 SDK 连接并持续发送零速。
- 只发送 `linear.x` 时直线前进或后退。
- 只发送 `angular.z` 时原地旋转。
- 同时发送两者时只执行原地旋转。
- 停止转向并进入角速度死区后，才开始执行直线速度。

## 验证结果

完成 `genisom_l1_control` 和 `scan_planner` 的构建、单元测试、launch 测试及代码风格检查：

```text
Summary: 94 tests, 0 errors, 0 failures, 11 skipped
```

其中 SDK 连接恢复依赖真实机器人心跳，需要在 NUC 与机器狗的实际网络环境中继续完成运行时确认。
