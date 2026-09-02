# Zsibot SDK API 参考文档

## 1. ZsibotExecutor 类介绍

**class ZsibotExecutor**

该类是与 Zsibot 四足机器人通信的主要接口类，采用 Pimpl 模式实现 ABI 稳定。内部通过三线程异步架构（发送、接收、心跳）与机器人进行 UDP 通信。

---

### 1.1 构造函数

```cpp
ZsibotExecutor(
    Role role = Role::ROLE_REMOTE,
    const std::string &send_ip = "192.168.234.1",
    uint32_t send_port = 8081,
    uint32_t recv_port = 8080
)
```

**功能概述**：初始化 Zsibot 执行器，启动三个工作线程（发送线程、接收线程、心跳线程）。

| 参数名       | 类型       | 说明          | 选项  | 备注                   |
| --------- | -------- | ----------- | --- | -------------------- |
| role      | Role     | 角色          | 可选项 | 默认值: ROLE_REMOTE     |
| send_ip   | string   | 发送数据的 IP 地址 | 可选项 | 默认值: "192.168.234.1" |
| send_port | uint32_t | 发送数据的端口号    | 可选项 | 默认值: 8081            |
| recv_port | uint32_t | 接收数据的端口号    | 可选项 | 默认值: 8080            |

**备注**：所有参数均为可选参数。如需指定后面的参数，必须按顺序提供前面的参数。

---

### 1.2 析构函数

```cpp
~ZsibotExecutor()
```

**功能概述**：释放网络连接和工作线程资源，等待所有线程安全结束。

---

### 1.3 检查连接状态

```cpp
bool IsConnected() const
```

**功能概述**：检查执行器是否已与机器人建立连接。

**返回值**：`true` 表示已连接（至少收到一个数据包且心跳在 5 秒内），`false` 表示未连接。

---

## 2. 控制接口

### 2.1 发送命令码

```cpp
void SetCmd(CmdCode cmd_code)
```

| 参数名      | 类型     | 说明                | 选项  |
| -------- | ------ | ----------------- | --- |
| cmd_code | CmdCode | 命令码，参见 [CmdCode 枚举](#31-cmdcode枚举) | 必选项 |

**备注**：发送预定义的设备命令。特技动作（跳跃、前跳、后空翻等）需先进入实验室模式。

---

### 2.2 设置遥控器输入

```cpp
void SetRemote(
    const std::array<float32_t, 4> &joy_stick,
    const std::array<float32_t, 14> &button
)
```

| 参数名       | 类型                        | 说明               | 选项  |
| --------- | ------------------------- | ---------------- | --- |
| joy_stick | std::array<float32_t, 4>  | 遥控杆数据 [前后, 旋转, 左右, 头部角度] | 必选项 |
| button    | std::array<float32_t, 14> | 按钮数据（14个float值） | 必选项 |

**备注**：摇杆值绝对值大于 1 时会自动归一化到 [-1, 1]。

---

### 2.3 设置 WiFi 信息

```cpp
void SetWifi(const WifiInfo &wifi_info)
```

| 参数名       | 类型       | 说明                                | 选项  |
| --------- | -------- | --------------------------------- | --- |
| wifi_info | WifiInfo | 包含 ssid 和 password 的 WifiInfo 结构体 | 必选项 |

---

## 3. 状态查询接口

### 3.1 基础信息

#### GetPower() — 获取电量百分比

```cpp
uint32_t GetPower() const
```
**返回值**：电量百分比（0-100）

#### GetTemperature() — 获取设备温度

```cpp
float32_t GetTemperature() const
```
**返回值**：设备温度（°C）

#### GetSn() — 获取设备序列号

```cpp
std::string GetSn() const
```
**返回值**：设备序列号字符串

#### GetDevName() — 获取设备名称

```cpp
std::string GetDevName() const
```
**返回值**：设备名称字符串

#### GetWifiSsid() — 获取 WiFi SSID

```cpp
std::string GetWifiSsid() const
```
**返回值**：当前 WiFi SSID

#### GetModel() — 获取设备型号

```cpp
Model GetModel() const
```
**返回值**：[Model](#35-model) 枚举值

---

### 3.2 速度与模式

#### GetSpeed() — 获取速度信息

```cpp
SpeedInfo GetSpeed() const
```
**返回值**：[SpeedInfo](#speedinfo) 结构体，包含速度、角速度、平移速度、头部角度。

#### GetSpeedLevel() — 获取速度等级

```cpp
SpeedLevel GetSpeedLevel() const
```
**返回值**：[SpeedLevel](#32-speedlevel) 枚举值

#### GetFunctionMode() — 获取功能模式

```cpp
FunctionMode GetFunctionMode() const
```
**返回值**：[FunctionMode](#33-functionmode) 枚举值

#### GetControlMode() — 获取控制模式

```cpp
ControlMode GetControlMode() const
```
**返回值**：[ControlMode](#34-controlmode) 枚举值

#### GetMotionMode() — 获取运动模式

```cpp
MotionMode GetMotionMode() const
```
**返回值**：[MotionMode](#36-motionmode) 枚举值，表示特技动作执行状态。

#### GetMotionType() — 获取运动类型

```cpp
MotionType GetMotionType() const
```
**返回值**：[MotionType](#37-motiontype) 枚举值，表示当前正在执行的特技类型。

#### GetVersion() — 获取版本信息

```cpp
VersionInfo GetVersion() const
```
**返回值**：[VersionInfo](#versioninfo) 结构体，包含主控版本和任务版本。

---

### 3.3 IMU 数据

#### GetQuaternion() — 获取四元数

```cpp
std::array<float32_t, 4> GetQuaternion() const
```
**返回值**：四元数 `[w, x, y, z]`

#### GetRPY() — 获取欧拉角

```cpp
std::array<float32_t, 3> GetRPY() const
```
**返回值**：欧拉角 `[Roll, Pitch, Yaw]`（弧度）

#### GetBodyAcc() — 获取机体加速度

```cpp
std::array<float32_t, 3> GetBodyAcc() const
```
**返回值**：机体坐标系加速度 `[x, y, z]`（m/s²）

#### GetBodyGyro() — 获取机体角速度

```cpp
std::array<float32_t, 3> GetBodyGyro() const
```
**返回值**：机体坐标系角速度 `[x, y, z]`（rad/s）

---

### 3.4 里程计数据

#### GetPosition() — 获取位置

```cpp
std::array<float32_t, 3> GetPosition() const
```
**返回值**：世界坐标系位置 `[x, y, z]`（米）

#### GetWorldVelocity() — 获取世界坐标系速度

```cpp
std::array<float32_t, 3> GetWorldVelocity() const
```
**返回值**：世界坐标系速度 `[x, y, z]`（m/s）

#### GetBodyVelocity() — 获取机体坐标系速度

```cpp
std::array<float32_t, 3> GetBodyVelocity() const
```
**返回值**：机体坐标系速度 `[x, y, z]`（m/s）

---

### 3.5 电机与关节数据

#### GetMotorTemp() — 获取电机温度

```cpp
std::array<float32_t, 16> GetMotorTemp() const
```
**返回值**：16 个电机的温度值（°C）

#### 关节角度（弧度）

| 方法 | 返回值 | 说明 |
|------|--------|------|
| GetLegAbadJoint() | array<float32_t, 4> | 四条腿髋外展关节角度 |
| GetLegHipJoint() | array<float32_t, 4> | 四条腿髋关节角度 |
| GetLegKneeJoint() | array<float32_t, 4> | 四条腿膝关节角度 |
| GetLegFootJoint() | array<float32_t, 4> | 四条腿足端关节角度 |

#### 关节角速度（rad/s）

| 方法 | 返回值 | 说明 |
|------|--------|------|
| GetLegAbadJointVel() | array<float32_t, 4> | 四条腿髋外展关节角速度 |
| GetLegHipJointVel() | array<float32_t, 4> | 四条腿髋关节角速度 |
| GetLegKneeJointVel() | array<float32_t, 4> | 四条腿膝关节角速度 |
| GetLegFootJointVel() | array<float32_t, 4> | 四条腿足端关节角速度 |

#### 关节力矩（N·m）

| 方法 | 返回值 | 说明 |
|------|--------|------|
| GetLegAbadJointTorque() | array<float32_t, 4> | 四条腿髋外展关节力矩 |
| GetLegHipJointTorque() | array<float32_t, 4> | 四条腿髋关节力矩 |
| GetLegKneeJointTorque() | array<float32_t, 4> | 四条腿膝关节力矩 |
| GetLegFootJointTorque() | array<float32_t, 4> | 四条腿足端关节力矩 |

> **关节数组顺序**：`[左前, 右前, 左后, 右后]`

---

### 3.6 故障与电池

#### GetFaultInfo() — 获取故障信息

```cpp
std::vector<FaultInfo> GetFaultInfo() const
```
**返回值**：[FaultInfo](#faultinfo) 列表，无故障时返回空列表。

#### GetBatteryInfo() — 获取电池详细信息

```cpp
BatteryInfo GetBatteryInfo() const
```
**返回值**：[BatteryInfo](#batteryinfo) 结构体。

---

### 3.7 WiFi 配置

#### GetWifiResutlf() — 获取 WiFi 配置结果

```cpp
std::optional<bool> GetWifiResutlf() const
```
**返回值**：`true` 成功，`false` 失败，`std::nullopt` 尚无结果。

#### IsRecvWifiResult() — 检查是否收到 WiFi 配置结果

```cpp
bool IsRecvWifiResult() const
```
**返回值**：`true` 表示已接收到结果。

---

## 4. 数据类型定义

### 4.1 CmdCode 枚举

| 枚举值                    | 十六进制  | 说明         |
| ---------------------- | ------ | ---------- |
| CMD_NULL               | 0x00   | 空命令        |
| CMD_EMERGENCY_STOP     | 0x5A   | 紧急停止       |
| CMD_STAND_UP           | 0x7A   | 站立         |
| CMD_SIT_DOWN           | 0x6A   | 趴下         |
| CMD_MOVE_MODE          | 0x8A   | 移动模式       |
| CMD_BALANCE_STAND_MODE | 0x9A   | 平衡站立模式     |
| CMD_LOCK_MODE          | 0x9B   | 锁定模式       |
| CMD_SLOW_SPEED         | 0xAE   | 慢速         |
| CMD_NORMAL_SPEED       | 0xAF   | 正常速度       |
| CMD_FAST_SPEED         | 0xB0   | 快速         |
| CMD_JUMP               | 0x01   | 跳跃         |
| CMD_FORWARD_JUMP       | 0x02   | 前跳         |
| CMD_BACK_FLIP          | 0x03   | 后空翻        |
| CMD_GREET              | 0x04   | 打招呼        |
| CMD_TWO_LEG_STAND      | 0x05   | 双腿站立       |
| CMD_CRAWL_FORWARD      | 0x09   | 匍匐前进       |
| CMD_CLIMBING_HIGH_PLATFORM | 0x0A | 爬高台       |
| CMD_HAND_STAND         | 0x0B   | 双腿倒立       |
| CMD_UNLOAD_SQUAT       | 0x0C   | 卸货下蹲       |
| CMD_ENTER_LAB_MODE     | 0xC1   | 进入实验室模式    |
| CMD_EXIT_LAB_MODE      | 0xC2   | 退出实验室模式    |
| CMD_ENTER_ENTERTAINMENT_MODE | 0xC3 | 进入文娱模式  |
| CMD_EXIT_ENTERTAINMENT_MODE  | 0xC4 | 退出文娱模式  |
| CMD_REMOTE_CONTROL_RIGHT     | 0xAA | 请求遥控控制    |
| CMD_SDK_CONTROL_RIGHT        | 0xB3 | 请求 SDK 控制  |
| CMD_ROAMERX_CONTROL_RIGHT    | 0xB4 | 请求漫游控制    |
| CMD_GENERAL_SDK_CONTROL_RIGHT | 0xB5 | 请求通用 SDK 控制 |

> **注意**：跳跃、前跳、后空翻、打招呼、双腿站立、卸货下蹲需要先调用 `CMD_ENTER_LAB_MODE` 进入实验室模式。
> 轮足机型（MODEL_XGW / MODEL_XGWHSPD）不能使用跳跃、前跳、打招呼、后空翻、双腿站立功能。

---

### 4.2 结构体

#### SpeedInfo

| 字段          | 类型       | 说明   |
| ----------- | -------- | ---- |
| speed       | float32_t | 前后速度 |
| angle_speed | float32_t | 角速度  |
| shift_speed | float32_t | 左右速度 |
| angle       | float32_t | 头部角度 |

#### VersionInfo

| 字段               | 类型     | 说明    |
| ---------------- | ------ | ----- |
| mc_version       | string | 运控版本  |
| dog_task_version | string | 任务版本  |

#### FaultInfo

| 字段         | 类型       | 说明   |
| ---------- | -------- | ---- |
| module     | string   | 故障所属模块 |
| submodule  | string   | 故障所属子模块 |
| error_code | uint32_t | 故障代码 |
| level      | string   | 故障等级 |
| info       | string   | 故障信息 |

#### WifiInfo

| 字段       | 类型     | 说明      |
| -------- | ------ | ------- |
| ssid     | string | WiFi 名称 |
| password | string | WiFi 密码 |

#### BatteryInfo

| 字段      | 类型       | 说明          |
| ------- | -------- | ----------- |
| current | float32_t | 电流 (A)      |
| error   | int32_t  | 错误码         |
| power   | int32_t  | 电量百分比       |
| temp    | float32_t | 温度 (°C)     |
| volt    | float32_t | 电压 (V)      |

---

### 4.3 枚举类型

#### SpeedLevel

| 枚举值       | 值  | 说明   |
| --------- | -- | ---- |
| SL_NULL   | 0  | 无效值  |
| SL_SLOW   | 1  | 慢速   |
| SL_NORMAL | 2  | 正常速度 |
| SL_FAST   | 3  | 快速   |

#### Model

| 枚举值          | 值  | 说明     |
| ------------ | -- | ------ |
| MODEL_XG      | 0  | 小狗点足   |
| MODEL_XGW     | 1  | 小狗轮足   |
| MODEL_XGWHSPD | 2  | 小狗高速轮足 |

#### FunctionMode

| 枚举值              | 值  | 说明      |
| ---------------- | -- | ------- |
| FM_NULL          | 0  | 无效值     |
| FM_REMOTE        | 1  | 遥控模式    |
| FM_TRACE         | 2  | 追踪模式    |
| FM_SDK           | 3  | SDK 模式  |
| FM_GENERAL_SDK   | 4  | 通用 SDK 模式 |
| FM_ROAMER        | 5  | 漫游模式    |

#### ControlMode

| 枚举值                   | 值  | 说明     |
| --------------------- | -- | ------ |
| CM_NULL               | 0  | 无效值    |
| CM_STAND_UP           | 1  | 站立     |
| CM_SIT_DOWN           | 2  | 趴下     |
| CM_LOCK_MODE          | 3  | 锁定模式   |
| CM_EMERGENCY_STOP     | 4  | 紧急停止   |
| CM_MOVE_MODE          | 5  | 移动模式   |
| CM_BALANCE_STAND_MODE | 6  | 平衡站立模式 |
| CM_INIT               | 7  | 初始化    |

#### MotionMode

| 枚举值        | 值  | 说明                |
| ---------- | -- | ----------------- |
| MM_NULL    | 0  | 无效值               |
| MM_RUNNING | 1  | 执行中（在进行特技动作）     |
| MM_REST    | 2  | 休息（未进行特技动作）       |
| MM_FORBID  | 3  | 禁止（不在实验室模式）       |

#### MotionType

| 枚举值                | 值  | 说明                |
| ------------------ | -- | ----------------- |
| MT_NULL            | 0  | 无效值               |
| MT_JUMP            | 1  | 跳跃                |
| MT_FORWARD_JUMP    | 2  | 前跳                |
| MT_BACK_FLIP       | 3  | 后空翻               |
| MT_GREET           | 4  | 招手                |
| MT_TWO_LEG_STAND   | 5  | 双腿站立              |
| MT_FORBID          | 6  | 禁止（未开启狂暴模式）       |
| MT_UNKNOWN         | 7  | 未知（未进行特技动作）       |
| MT_UNLOADING_SQUAT | 8  | 卸货下蹲              |

#### Role

| 枚举值         | 值  | 说明   |
| ----------- | -- | ---- |
| ROLE_NULL   | 0  | 无效值  |
| ROLE_REMOTE | 1  | 遥控用户 |
| ROLE_SDK    | 2  | SDK 用户 |
