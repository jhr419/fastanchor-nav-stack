# GENISOM L1-W Twist 控制使用说明

本说明用于 `twist.launch.py` 独立速度控制模式。该模式启动后自动申请 SDK 控制权、站立、进入移动模式，并接收 ROS2 `/cmd_vel`。

## 1. 使用前确认

- Firefly 地址为 `192.168.168.168`，NUC 能够 ping 通该地址。
- 机器人位于平坦、空旷场地，遥控器和原厂急停可用。
- 不要同时启动 `manager.launch.py`、官方 SDK 示例或其他占用 UDP 8080 的程序。
- 本节点只执行前后移动 `linear.x` 和转向 `angular.z`，横移 `linear.y` 始终被强制为零。

## 2. 首次编译

```bash
cd zs_sdk
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select genisom_l1_control
source install/setup.bash
```

## 3. 启动控制节点

终端 1：

```bash
cd zs_sdk
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch genisom_l1_control twist.launch.py
```

节点将自动完成：

1. 连接官方 SDK；
2. 请求 SDK 控制权；
3. 控制机器人站立；
4. 进入移动模式；
5. 开始接收 `/cmd_vel`。

## 4. 确认是否就绪

终端 2：

```bash
cd zs_sdk
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic echo --once /genisom/twist/ready
```

看到以下结果后才能发送速度：

```text
data: true
```

查看详细状态：

```bash
ros2 topic echo /genisom/twist/status
```

## 5. 发送速度

低速前进并轻微转向：

```bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.02, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.1}}'
```

停止发送可按 `Ctrl+C`。节点超过 300 ms 没有收到新 `/cmd_vel` 时会自动发送零速停车。

使用键盘控制：

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

导航程序也可以直接向 `/cmd_vel` 发布。即使上层发送非零 `linear.y`，SDK 下发的横向速度仍固定为零。

## 6. 遥控器接管

需要人工接管时，在遥控器上长按 `L2 + R2 + A` 两秒。

机器人反馈切换到 REMOTE 后，节点会：

- 停止接收和下发 `/cmd_vel`；
- 不再申请 SDK 控制权；
- 退出进程并释放 SDK 连接。

接管后如需恢复 ROS2 控制，请先确认现场安全，然后重新运行：

```bash
ros2 launch genisom_l1_control twist.launch.py
```

## 7. 正常退出

在启动节点的终端按 `Ctrl+C`。节点会停车并请求切回 REMOTE。

## 8. 常见问题

### 一直显示 `model is empty, please wait..`

先检查网络和路由：

```bash
ping -c 3 192.168.168.168
ip route get 192.168.168.168
```

再检查是否有其他 SDK 程序占用连接或 UDP 8080：

```bash
pgrep -af 'genisom|sdk_control|remote_control'
ss -lunp | grep ':8080'
```

### `/genisom/twist/ready` 为 `false`

说明尚未完成 SDK 控制权、站立或移动模式确认。不要发送运动命令，查看 `/genisom/twist/status` 和启动终端日志。

### 发送 `/cmd_vel` 后不运动

依次确认：

```bash
ros2 topic echo --once /genisom/twist/ready
ros2 topic hz /cmd_vel
ros2 topic echo /cmd_vel
```

`ready` 必须为 `true`，并且 `/cmd_vel` 需要持续发布，不能只发送一次。

### 遥控器接管后节点退出

这是预期行为。节点不会自动抢回 SDK 控制权，需要人工重新启动。
