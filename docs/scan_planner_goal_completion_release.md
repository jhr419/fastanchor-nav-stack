# SCAN-Planner 到达终点后释放导航目标修改记录

## 修改背景

Waypoint Action 到达最后一个目标点后虽然已经返回成功，但闭环控制器仍然保存最后一条 B-spline。机器人被遥控移开终点后，控制器检测到终点位置误差重新增大，因此会再次输出速度并返回最后一个目标点。

问题位于上层任务和 SCAN-Planner 控制链路，与宇树或智身底盘 SDK 无关。底盘只是在执行持续收到的 `cmd_vel`。

## 修改内容

### 1. 增加轨迹取消命令

修改 `src/SCAN-Planner/planner/scan_planner_msgs/msg/Bspline.msg`：

- 增加 `EXECUTE`、`CANCEL` 和 `HOLD` 三种命令常量。
- 增加 `command` 字段。
- 默认值 `0` 对应 `EXECUTE`，已有未显式设置该字段的轨迹发布端仍按执行轨迹处理。

取消命令与执行命令复用 `planning/bspline`，从而保持同一发布端、同一 topic 内的消息顺序，避免独立取消 topic 与旧轨迹之间出现时序竞争。

### 2. SCAN 停止时显式作废轨迹

修改：

- `src/SCAN-Planner/planner/plan_manage/include/plan_manage/scan_replan_fsm.h`
- `src/SCAN-Planner/planner/plan_manage/src/scan_replan_fsm.cpp`

`/scan_planner/set_navigation_enabled=false` 现在会：

- 禁止接收迟到的目标和路径。
- 清除目标、触发器、活动路径点和重规划状态。
- 发布 `CANCEL` 命令，让控制器彻底释放旧 B-spline。
- 将 FSM 切换到 `WAIT_TARGET`。

原来的停止方式会发布当前位置的静止 B-spline，该轨迹仍然是一个位置目标。修改后不再用位置保持轨迹表达任务结束。重复请求关闭导航时也会重新发布取消命令，保证停止操作可以安全重试。

紧急停车仍然发布带有 `HOLD` 命令的静止 B-spline，因为紧急状态需要在重新规划前主动保持停车位置；普通轨迹使用 `EXECUTE`，任务完成、暂停和取消则使用 `CANCEL` 释放位置目标。

### 3. 控制器释放最后轨迹

修改：

- `src/SCAN-Planner/planner/plan_manage/src/closed_loop_controller.cpp`
- `src/SCAN-Planner/planner/plan_manage/src/open_loop_controller.cpp`

闭环控制器收到 `CANCEL` 后会：

- 将 `receive_traj_` 设置为 `false`。
- 清空 B-spline 和轨迹时间状态。
- 清除执行冻结状态。
- 立即发布零速度，并在没有新轨迹时持续发布零速度。

闭环控制器正常运行到轨迹末端且进入 `finish_dist` 后，也会主动释放轨迹。这样即使不是通过 Waypoint Action 发送的单目标任务，到点后也不会恢复跟踪旧终点。

开环控制器同步支持 `CANCEL`，确保仿真与真实闭环模式具有一致的任务结束语义。

### 4. Action 成功前先停止 SCAN

修改 `src/navigation_bringup/navigation_bringup/waypoint_mission_manager.py`：

- 最后一个路径点到达后先进入 `COMPLETING`，不立即返回 Action 成功。
- 调用 `/scan_planner/set_navigation_enabled=false` 并等待响应。
- SCAN 确认停止后，将任务置为 `COMPLETED` 并返回 `SUCCEEDED`。
- 如果停止服务不可用、超时或返回失败，Action 返回失败，避免控制器仍可能运动时向调用方报告任务成功。

暂停仍会保留任务路径点索引。恢复时现有逻辑会重新启用 SCAN，并重新发布当前路径点以生成全新轨迹。新的 Action 开始时也会重新启用 SCAN。

### 5. 单目标到达后清除 A* 活动目标和缓存路径

仅释放闭环控制器中的最后一条 B-spline 仍然不够。A* 原来会一直保存 `active_goal_`，并在机器人被移出旧目标点后根据起点变化重新规划；全局路径裁剪节点也会持续发布缓存的旧路径。

修改：

- `src/FastPlanner/src/3dnav_global_planning/src/astar_global_planner_node.cpp`
- `src/FastPlanner/src/3dnav_global_planning/config/astar_global_planner.yaml`
- `src/SCAN-Planner/planner/plan_manage/src/global_path_window_node.cpp`

A* 现在只对已经成功发布过路径的活动目标执行到达判定。当机器人 XY 位置进入 `goal_reached_tolerance` 后，会：

- 清除 `active_goal_`、待规划目标和上次规划起点；
- 清除目标请求去重状态，允许之后重新发送同一坐标；
- 发布空的全局路径，覆盖持久化的旧路径；
- 删除 RViz 中的 A* 全局路径标记；
- 将状态设置为 `GOAL_REACHED`。

全局路径裁剪节点收到空全局路径后会同步发布一次空局部路径并停止更新，从而保证旧路径不会继续进入 SCAN-Planner。到达后再手动移开机器人，不会重新规划到已经完成的目标。

新增参数：

```yaml
clear_goal_on_reach: true
goal_reached_tolerance: 0.15
```

## 到达阈值说明

Waypoint Action 使用 `xy_tolerance` 和 `hold_time` 判断任务完成。Action 满足该条件后会立即取消控制轨迹，因此实际任务结束精度由这些参数决定。

闭环控制器的 `finish_dist` 用于没有 Action 参与时的轨迹自动释放。A* 单目标模式使用 `goal_reached_tolerance` 清除全局活动目标。当前两个参数均为 `0.15 m`，应保持相同或相近，避免全局目标和局部轨迹使用明显不同的完成范围。

## 回归测试

扩展 `src/SCAN-Planner/planner/plan_manage/test/test_closed_loop_constraints.py`，覆盖以下行为：

- 非完整运动学模式仍只输出 `linear.x` 和 `angular.z`。
- 活动轨迹被取消后，即使里程计位置被移动，控制器仍持续输出零速度。
- 取消后发布新轨迹可以重新启动控制。
- 静止轨迹正常完成后会自动释放，随后移动里程计位置也不会恢复旧目标跟踪。
- `HOLD` 轨迹结束后仍保持目标，用于验证紧急停车的位置保持语义没有被削弱。
- 空全局路径会同步清除路径裁剪节点的局部路径缓存。
- A* 进入 `0.15 m` 到达范围后会发布空路径和 `GOAL_REACHED`，移出目标点后不会重新规划旧目标。

建议构建命令：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select scan_planner_msgs scan_planner nav3d_global_planning navigation_bringup
```

建议测试命令：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon test --packages-select scan_planner nav3d_global_planning navigation_bringup --event-handlers console_direct+
colcon test-result --verbose
```

本次修改已使用上述构建和测试流程完成验证，结果为：

```text
Summary: 29 tests, 0 errors, 0 failures, 0 skipped
```

由于 `Bspline.msg` 接口发生了变化，部署时必须重新构建 `scan_planner_msgs`、`scan_planner` 以及所有使用该消息的下游节点，不应只替换单个可执行文件。
