# 官方 genisom_L1_sdk 分析

## 分析范围与版本

分析对象为 `third_party/genisom_L1_sdk`，来源是官方 `zsibot/genisom_L1_sdk` 的 `main` 分支。

```text
提交: f7ccbf393e96f1205af8cce8070c5886f9641428
日期: 2026-06-22T18:48:55+08:00
主题: first commit
```

本轮实际检查了：

- `README.md`、`README_en.md`、`CHANGELOG.md`
- `CMakeLists.txt`
- `example/CMakeLists.txt`
- `example/remote_control.cpp`
- `example/sdk_control.cpp`
- `include/zsibot_sdk/zsibot_api.h`
- `include/zsibot_sdk/zsibot_define.h`
- `include/zsibot_sdk/zsibot_exception.h`
- `include/zsibot_sdk/lowlevel/lowlevel.h`
- `docs/zh/api.md`、`docs/zh/api_lowlevel.md`
- `docs/en/api.md`、`docs/en/api_lowlevel.md`
- `docs/protocol/protocol.md`
- `lib/x86_64/libzsibot.a`、`lib/aarch64/libzsibot.a`

仓库不包含高层 SDK 的 `.cpp` 源码，只包含公开头文件和预编译静态库。因此可准确分析公开 API、示例、协议和
导出符号，但不能审计 UDP 线程内部实现、命令队列策略或线程安全细节。

## 构建要求

`CMakeLists.txt:9-10` 强制：

```cmake
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)
```

虽然 `README.md:51` 和 `CHANGELOG.md:50` 写“C++17 编译器”，但 `README.md:3` 又写“基于 C++20”。实际
构建配置是可执行依据，因此适配包使用 C++20。

根项目通过架构选择预编译静态库：

- `CMakeLists.txt:20-25`：aarch64/arm64 选择 `aarch64`，其他默认选择 `x86_64`
- `CMakeLists.txt:28-39`：将 `lib/<架构>/libzsibot.a` 声明为 IMPORTED 静态库
- `CMakeLists.txt:47`：接口依赖 `Threads::Threads`

项目版本也存在文档差异：根 CMake 为 `1.1.0`，`CHANGELOG.md` 只有 `1.0.0` 条目。升级判断应以 Git 提交
和实际 API 为准，不能只依赖 changelog 版本。

## ZsibotExecutor 初始化与退出

公开声明位于 `include/zsibot_sdk/zsibot_api.h:19-21`：

```cpp
ZsibotExecutor(
  Role role = Role::ROLE_REMOTE,
  const std::string & send_ip = "192.168.234.1",
  uint32_t send_port = 8081,
  uint32_t recv_port = 8080);
~ZsibotExecutor();
```

结论：

| 项目 | 实际设计 |
|---|---|
| 默认角色 | `Role::ROLE_REMOTE` |
| ROS 适配层角色 | 必须显式使用 `Role::ROLE_SDK` |
| 默认机器人 IP | `192.168.234.1` |
| 默认发送端口 | `8081` |
| 默认接收端口 | `8080` |
| 显式 Connect/Start | 不存在 |
| 建立通信 | 构造函数内部初始化 UDP 并启动工作线程 |
| 安全退出 | 析构函数释放网络和等待工作线程退出 |

官方 `docs/zh/api.md:22` 说明构造函数启动发送、接收、心跳三个线程，`docs/zh/api.md:41` 说明析构函数
等待线程安全结束。静态库导出符号中能确认构造、析构、`SetCmd`、`SetRemote`、`IsConnected` 和
`GetFunctionMode` 的实现存在，但内部代码不可见。

### 预编译库实际存在构造阻塞

无机器人冒烟测试与静态库反汇编发现，文档没有描述完整构造行为：

1. `ZsibotExecutorImpl` 构造函数绑定接收端口后调用内部 `Start()`；
2. `Start()` 依次创建 `HeartbeatThread`、`RecvThread`、`SendThread`；
3. 随后在当前构造线程中循环调用 `ValidateModel(CmdCode::CMD_NULL)`；
4. 校验失败时执行 `nanosleep` 并重试，运行时输出 `model is empty, please wait..`；
5. 只有收到有效 model 后 `Start()` 才返回，外层 `ZsibotExecutor` 构造才完成。

该结论来自 `lib/x86_64/libzsibot.a` 中未剥离的符号及 `zsibot_executor_impl.cpp.o` 反汇编，不是根据函数名猜测。
无机器人测试中，进程收到 SIGINT 后 5 秒未退出，收到 SIGTERM 后再等 10 秒仍未退出，最终由 ROS launch
发送 SIGKILL。循环中没有发现构造取消或运行标志检查。

适配层不在线程内包装这个不可取消构造，而是在调用 SDK 之前 fork 独立工作进程。外层监督进程转发
SIGINT/SIGTERM，保留 3 秒用于正常停车、释放 REMOTE 和析构；若构造仍阻塞，则只对工作进程发送 SIGKILL
并等待回收。2026-08-26 使用不可达地址和隔离端口回归，单次 SIGINT 后进程按预期在 3 秒后以 130 退出，
没有残留控制节点进程。

影响：

- 首次启动时机器人必须在线并能返回 model；
- 节点不能在构造完成前发布 `DISCONNECTED`；
- 正常析构安全策略只适用于 SDK 已完成构造的情况；
- `genisom_manager_node` 与 `genisom_twist_node` 的 SDK 工作子进程同样受影响，但会被监督进程回收；
- 首次成功连接后的后续掉线仍可用公开 `IsConnected()` 检测。

不采用后台 detach 线程包装构造，因为不可取消的构造线程在节点退出后继续访问对象会造成生命周期未定义行为；
也不修改预编译库或自行伪造 model。正确的上游修复应让 `Start()` 异步返回，或提供可取消/带超时的 Connect。

## Role 的准确枚举

定义位于 `include/zsibot_sdk/zsibot_define.h:143-148`：

| 枚举 | 值 | 含义 |
|---|---:|---|
| `ROLE_NULL` | 0 | 无效 |
| `ROLE_REMOTE` | 1 | 遥控用户 |
| `ROLE_SDK` | 2 | SDK 用户 |

Role 是客户端在心跳和命令中的身份，不等同于机器人当前控制权。`Role::ROLE_SDK` 的程序仍需发送
`CMD_SDK_CONTROL_RIGHT`，并等待机器人反馈 `FM_SDK` 后才可运动。

## UDP 与连接状态

`docs/protocol/protocol.md:5-13` 定义 JSON over UDP，默认网络为：

| 参数 | 默认值 |
|---|---|
| Robot IP | `192.168.234.1` |
| Send Port | `8081` |
| Recv Port | `8080` |

协议文档 `:21-25` 显示心跳每 50 ms 发送一次，超时 5 秒。`docs/protocol/protocol.md:124-129` 进一步说明：

```text
IsConnected() == true
  当且仅当至少收到过一个机器人数据包，且最近心跳/数据时间在 5 秒内
```

公开 API 是 `include/zsibot_sdk/zsibot_api.h:23` 的 `bool IsConnected() const`。没有单独的连接回调或
重连 API；适配层通过定时器查询 SDK 自带连接状态，不重复实现 UDP 心跳。

## 控制权命令

准确枚举位于 `include/zsibot_sdk/zsibot_define.h:53-56`：

| 命令 | 值 | 目标 FunctionMode |
|---|---:|---|
| `CMD_REMOTE_CONTROL_RIGHT` | `0xAA` | `FM_REMOTE` |
| `CMD_SDK_CONTROL_RIGHT` | `0xB3` | `FM_SDK` |
| `CMD_ROAMERX_CONTROL_RIGHT` | `0xB4` | `FM_ROAMER` |
| `CMD_GENERAL_SDK_CONTROL_RIGHT` | `0xB5` | `FM_GENERAL_SDK` |

调用形式均为：

```cpp
executor.SetCmd(zsibot::CmdCode::CMD_SDK_CONTROL_RIGHT);
```

状态查询的实际 API 确实是 `GetFunctionMode()`，声明位于 `include/zsibot_sdk/zsibot_api.h:38`。
`FunctionMode` 位于 `include/zsibot_sdk/zsibot_define.h:99-107`：

| 枚举 | 值 | 含义 |
|---|---:|---|
| `FM_NULL` | 0 | 无效/尚无反馈 |
| `FM_REMOTE` | 1 | 遥控模式 |
| `FM_TRACE` | 2 | 追踪模式 |
| `FM_SDK` | 3 | SDK 模式 |
| `FM_GENERAL_SDK` | 4 | 通用 SDK 模式 |
| `FM_ROAMER` | 5 | 漫游模式 |

`docs/protocol/protocol.md:114` 明确要求 SDK 用户先申请控制权，并依据反馈确认切换，而不是把发命令视为立即成功。

### REMOTE 到 SDK

```text
构造 Role::ROLE_SDK 的 ZsibotExecutor
  -> 等待 IsConnected() == true
  -> 确认 GetFunctionMode() == FM_REMOTE
  -> SetCmd(CMD_SDK_CONTROL_RIGHT)
  -> 轮询 GetFunctionMode()
  -> 只有确认 FM_SDK 后才发送零摇杆
  -> 清空旧 cmd_vel，等待新消息
```

### SDK 到 REMOTE

```text
FM_SDK
  -> SetRemote({0,0,0,0}, 全零按钮) 一次
  -> SetCmd(CMD_REMOTE_CONTROL_RIGHT)
  -> 停止持续发送 SetRemote
  -> 轮询 GetFunctionMode()
  -> 确认 FM_REMOTE 后报告 control_owner=REMOTE
```

官方两个示例都允许发送四种控制权命令：`example/remote_control.cpp:185-199` 和
`example/sdk_control.cpp:164-178`。Manager 为四种模式分别提供 service，但只有 `FM_SDK` 允许速度桥下发；
进入 GENERAL_SDK 或 ROAMERX 时遥测继续发布，速度桥保持关闭。

## 新版 L1-W 运动控制 API

高层运动接口实际仍命名为 `SetRemote`：

```cpp
void SetRemote(
  const std::array<float32_t, 4> & joy_stick,
  const std::array<float32_t, 14> & button);
```

声明见 `include/zsibot_sdk/zsibot_api.h:26`，详细顺序见 `docs/zh/api.md:73-87` 和
`docs/protocol/protocol.md:33-45`：

```text
joy_stick = [前后, 旋转, 左右, 头部角度]
```

官方示例提供了符号依据：

- `example/sdk_control.cpp:96-101`：第 0 维正值前进、负值后退
- `example/sdk_control.cpp:104-109`：第 2 维正值左移、负值右移
- `example/sdk_control.cpp:112-117`：第 1 维正值左转、负值右转
- `example/sdk_control.cpp:120-122`：全零摇杆停车

ROS 映射为：

| ROS | GENISOM | 单位/范围 |
|---|---|---|
| `linear.x` | `joy_stick[0]` | ROS 输入 m/s；SDK 输入归一化摇杆 |
| `linear.y` | `joy_stick[2]` | ROS 输入 m/s；SDK 输入归一化摇杆 |
| `angular.z` | `joy_stick[1]` | ROS 输入 rad/s；SDK 输入归一化摇杆 |
| 无 | `joy_stick[3]` | 固定 0 |

### 重要限制：没有物理速度标定曲线

`docs/zh/api.md:87` 只说明绝对值超过 1 会归一化到 `[-1,1]`。SDK 的 `GetSpeed()` 返回 m/s 和 rad/s，
但仓库未给出“摇杆值到实际速度”的换算、L1-W 各速度档上限或线性保证。因此不能声称
`joy_stick[0]=0.1` 等于 `0.1 m/s`。

Manager 将限速和单位换算拆开：

1. `limit_cmd_vel_input=false` 时由导航器/规划器负责 ROS 物理速度限制；设为 `true` 后才使用
   `max_linear_x/y`、`max_angular_z` 做末端限幅；
2. `forward/lateral_joystick_per_mps` 和 `yaw_joystick_per_rps` 独立定义物理速度到归一化摇杆的标定增益；
3. 输出始终按 `max_forward/lateral/yaw_joystick` 截断，默认 `1.0`，对应官方协议边界。

标定增益不代表官方保证的线性速度曲线。实际速度应读取 `GetSpeed()` 或外部里程计，在封闭场地逐级测量。

### 旧版 ZSL-1W highlevel 直接速度接口

旧版 `genisom_l1_sdk_old` 的 ZSL-1W highlevel 接口另提供：

```cpp
uint32_t move(const float vx, const float vy, const float yaw_rate);
```

文档 `third_party/genisom_l1_sdk_old/docs/api_zsl-1w.md` 将参数定义为前向速度 m/s、侧向速度 m/s 和绕 Z 轴角速度
rad/s。`include/zsl-1w/highlevel.h` 与对应动态库也导出了该接口。

因此独立 `genisom_twist_node` 已改为使用旧 highlevel 后端：

```text
/cmd_vel -> HighLevel::move(linear.x, linear.y, angular.z)
```

Twist 模式不再通过新版 `SetRemote()` 下发速度，也不再做归一化摇杆换算、yaw 优先互斥、固定频率重发或
300 ms 超时清零。旧版 highlevel 没有新版 `GetFunctionMode()`，所以 Twist 模式无法确认遥控器接管状态。

## 机型识别与轮足约束

`Model` 定义位于 `include/zsibot_sdk/zsibot_define.h:92-97`：

| 枚举 | 含义 |
|---|---|
| `MODEL_XG` | 点足 |
| `MODEL_XGW` | 轮足，L1-W 预期属于此类 |
| `MODEL_XGWHSPD` | 高速轮足 |

官方 `CHANGELOG.md:40-44` 和 `docs/zh/api.md:371-372` 说明轮足机型不能执行跳跃、前跳、招手、后空翻和
双腿站立。Manager 暴露官方动作服务，但每次执行前调用 `GetModel()`：轮足/高速轮足自动拒绝上述动作，点足
自动拒绝匍匐、爬高台和双腿倒立。卸货下蹲按官方说明要求先进入实验室模式。

实机第一次连接必须记录 `GetModel()` 的实际返回，不能只根据产品名称推定枚举。

## 急停与恢复

官方急停命令只有：

```cpp
executor.SetCmd(zsibot::CmdCode::CMD_EMERGENCY_STOP);
```

- 枚举：`include/zsibot_sdk/zsibot_define.h:31`
- 示例：`example/sdk_control.cpp:124-126`
- 示例描述：软急停、进入失能状态、完全爬下
- 状态反馈：`ControlMode::CM_EMERGENCY_STOP`，定义于 `zsibot_define.h:115`

仓库中不存在 `ClearEmergencyStop`、`CMD_CLEAR_EMERGENCY_STOP` 或其他独立解除急停 API。`CMD_STAND_UP`
虽然存在，但官方资料没有说明它是急停解除命令，所以适配层不会用站立命令冒充清急停。

最终策略：`emergency_stop` 发送官方命令并锁存 ROS ESTOP；`clear_emergency_stop` 只请求并确认 REMOTE，
然后由操作员使用原厂遥控器和原厂恢复流程处置。

## 异常与线程安全

`include/zsibot_sdk/zsibot_exception.h:10-23` 定义 `ZsibotException`，继承 `std::exception`。公开 API 没有
声明 noexcept，也没有列出各方法的具体异常条件，因此适配层在控制与查询边界捕获 `std::exception`。

官方资料说明 SDK 内部有三个异步线程，但没有承诺 `ZsibotExecutor` 的公开方法可由多个调用线程并发调用。
Manager 在 `SdkWrapper` 中用一个互斥锁串行化所有新版 SDK getter 和 setter。Twist 在
`HighLevelVelocityClient` 中串行化旧 highlevel 连接、站立和 `move()` 调用。ROS 默认使用单线程 executor，
互斥锁仍作为未来切换 executor 时的防护。

## 本轮没有修改官方 SDK

`third_party/genisom_L1_sdk` 的 Git 工作区保持干净。构建产物位于工作区的 `build/official_sdk`，没有写入官方
仓库目录。自研 wrapper 位于 `src/genisom_l1_control`。

## 仍需官方或实机确认的问题

1. 已确认该 L1-W 实机报告 `MODEL_XGWHSPD`，但新 Manager 仍需再次记录。
2. 三个归一化摇杆轴在各速度档下对应的 m/s、rad/s 曲线和死区。
3. `CMD_EMERGENCY_STOP` 在 REMOTE 与 SDK FunctionMode 下是否都接受，以及反馈延迟。
4. 急停后的官方推荐恢复步骤。
5. 四种控制权请求的拒绝条件和典型确认延迟。
6. 多次 REMOTE -> SDK -> REMOTE 后是否存在固件冷却时间或命令频率限制。
7. SDK 断网重连时内部 UDP socket 和线程是否自动恢复；公开 API 没有重建连接接口。
8. IMU、odom、关节和 16 路温度的坐标、零点、源时间戳与实机对应关系。
9. 上游是否能修复构造期 `ValidateModel` 无限等待，并提供可取消或超时参数。

这些问题必须在 `docs/TEST_PLAN.md` 留下实测结果，未验证前不能对外宣称控制适配层已经通过实机验收。
