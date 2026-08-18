# 新导航系统使用说明

## 1. 加载 ROS2 与工作空间环境

```bash
cd ~/workspace/fastanchor-nav-stack
source /opt/ros/humble/setup.bash
source install/setup.bash
```

说明：每个新终端都需要执行一次。

---

## 2. 启动完整导航系统

```bash
cd ~/workspace/fastanchor-nav-stack
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch navigation_bringup navigation_system.launch.py
```

说明：`waypoint_mission_manager` 会随系统自动启动，不再需要提供 waypoint YAML 文件。

---

## 3. 检查任务管理接口

### 检查任务管理节点

```bash
ros2 node list | grep waypoint
```

应看到：

```text
/waypoint_mission_manager
```

### 检查动态航点 Action

```bash
ros2 action info /follow_waypoints
```

应看到：

```text
Action servers: 1
    /waypoint_mission_manager
```

### 检查暂停/恢复 Service

```bash
ros2 service type /waypoint_mission/control
```

应输出：

```text
nav_interfaces/srv/ControlMission
```

### 检查 SCAN-Planner 内部启停接口

```bash
ros2 service type /scan_planner/set_navigation_enabled
```

应输出：

```text
std_srvs/srv/SetBool
```

---

## 4. 查看当前任务状态

查看一次：

```bash
ros2 topic echo /waypoint_mission/status --once
```

持续查看：

```bash
ros2 topic echo /waypoint_mission/status
```

空闲状态示例：

```text
data: '{"mission_id":"","state":"IDLE","current_index":null,"waypoint_count":0,"progress":0.0,"paused":false}'
```

常见状态：

```text
IDLE
STARTING
WAITING_FOR_ODOMETRY
WAITING_FOR_PATH
NAVIGATING
PAUSING
PAUSED
RESUMING
CANCELING
COMPLETED
SUCCEEDED
CANCELED
FAILED
```

---

## 5. 动态下发一组 `[x,y,z]` 航点

示例：

```bash
ros2 action send_goal \
/follow_waypoints \
nav_interfaces/action/FollowWaypoints \
"{
  mission_id: 'test_001',
  frame_id: 'map',
  waypoints: [
    {x: -0.8, y: 0.64, z: 0.0},
    {x: -1.8, y: 0.64, z: 0.0}
  ]
}" \
--feedback
```

参数说明：

- `mission_id`：本次任务唯一名称。
- `frame_id`：当前使用 `map`。
- `waypoints`：按执行顺序填写 `[x,y,z]` 坐标，单位为米。
- `--feedback`：持续显示当前航点、进度和任务状态。

正常状态变化：

```text
WAITING_FOR_PATH
-> NAVIGATING
-> 下一个 waypoint
-> ...
-> COMPLETED / SUCCEEDED
```

`current_index` 从 `0` 开始。

---

## 6. 暂停当前任务

假设当前任务：

```text
mission_id = test_001
```

执行：

```bash
ros2 service call \
/waypoint_mission/control \
nav_interfaces/srv/ControlMission \
"{mission_id: 'test_001', command: 1}"
```

说明：

```text
command: 1 = PAUSE
```

正常状态：

```text
NAVIGATING
-> PAUSING
-> PAUSED
```

暂停后：

- 机器人停止运动。
- 当前 `mission_id` 保留。
- 当前 `current_index` 保留。
- 剩余 waypoint 保留。
- `/follow_waypoints` Action 不结束。

---

## 7. 恢复当前任务

执行：

```bash
ros2 service call \
/waypoint_mission/control \
nav_interfaces/srv/ControlMission \
"{mission_id: 'test_001', command: 2}"
```

说明：

```text
command: 2 = RESUME
```

正常状态：

```text
PAUSED
-> RESUMING
-> WAITING_FOR_PATH
-> NAVIGATING
```

恢复时不会继续使用暂停前的旧轨迹，而是：

```text
机器人当前位置
-> 重新发布当前 waypoint
-> FastPlanner 重新规划
-> 新 /planned_path
-> SCAN-Planner 继续执行
```

---

## 8. 直接停止/恢复 SCAN-Planner 执行

> 此接口主要供 `waypoint_mission_manager` 内部使用，正常中控操作优先使用 `/waypoint_mission/control`。

### 停止 SCAN-Planner

```bash
ros2 service call \
/scan_planner/set_navigation_enabled \
std_srvs/srv/SetBool \
"{data: false}"
```

作用：停止当前 SCAN 执行并忽略后续旧路径。

### 重新允许 SCAN-Planner 执行

```bash
ros2 service call \
/scan_planner/set_navigation_enabled \
std_srvs/srv/SetBool \
"{data: true}"
```

作用：只恢复执行权限，不会自动恢复旧轨迹；上层需要重新发布目标。

---

## 9. 查看全局规划路径

查看一次：

```bash
ros2 topic echo /planned_path --once
```

说明：FastPlanner 接收到当前 waypoint 后，会发布新的全局路径到 `/planned_path`。

---

## 10. 查看机器人速度指令

```bash
ros2 topic echo /cmd_vel
```

说明：用于确认机器人当前是否仍有运动控制输出。

---

## 11. Action 接口定义

```bash
ros2 interface show nav_interfaces/action/FollowWaypoints
```

当前接口：

```text
# Goal
string mission_id
string frame_id
geometry_msgs/Point[] waypoints

---
# Result
bool success
string message

---
# Feedback
uint32 current_index
uint32 total_waypoints
float32 progress
string state
```

---

## 12. Pause / Resume 接口定义

```bash
ros2 interface show nav_interfaces/srv/ControlMission
```

当前接口：

```text
uint8 PAUSE=1
uint8 RESUME=2

string mission_id
uint8 command

---
bool success
string message
string state
```

---

## 13. 当前推荐操作流程

```text
1. 启动 navigation_system.launch.py
2. 确认 /follow_waypoints 存在
3. 通过 /follow_waypoints 下发 waypoint 点集
4. 通过 /waypoint_mission/status 查看状态
5. command=1 暂停
6. command=2 恢复
7. 等待任务完成
```

## 14. Cancel

`/follow_waypoints` 已按 ROS2 Action 设计取消逻辑。中控 Action Client 应保存返回的 `goal_handle`，取消时调用：

```python
goal_handle.cancel_goal_async()
```

取消链路：

```text
CANCELING
-> SCAN-Planner 停止
-> Action CANCELED
-> mission 清理
-> IDLE
```

当前命令行主要用于 `send / pause / resume` 测试；正式中控端建议通过 ROS2 Action Client 实现 Cancel。
