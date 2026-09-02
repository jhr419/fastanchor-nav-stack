# Zsibot SDK API Reference

## 1. ZsibotExecutor Class

The main interface class for communicating with Zsibot robot dogs via UDP protocol. Uses the Pimpl pattern for ABI stability.

### 1.1 Constructor

```cpp
ZsibotExecutor(role = ROLE_REMOTE, send_ip = "192.168.234.1", send_port = 8081, recv_port = 8080)
```

| Parameter | Type     | Description              | Default            |
|-----------|----------|--------------------------|--------------------|
| role      | Role     | User role                | ROLE_REMOTE        |
| send_ip   | string   | Robot IP address         | "192.168.234.1"    |
| send_port | uint32_t | UDP send port            | 8081               |
| recv_port | uint32_t | UDP receive port         | 8080               |

All parameters are optional. Three worker threads (send, receive, heartbeat) are started on construction.

### 1.2 Destructor

```cpp
~ZsibotExecutor()
```

Cleans up resources and waits for all worker threads to finish.

### 1.3 Connection Status

```cpp
bool IsConnected() const
```
Returns `true` if connected to the robot (at least one packet received and heartbeat within 5 seconds).

### 1.4 Control Commands

```cpp
void SetCmd(CmdCode cmd_code)
```
Send a command code to the robot. See [CmdCode](#21-cmdcode) for available values.

```cpp
void SetRemote(const std::array<float32_t, 4>& joy_stick, const std::array<float32_t, 14>& button)
```
Set remote control input. Joystick values with absolute value > 1 are automatically normalized.

```cpp
void SetWifi(const WifiInfo& wifi_info)
```
Configure WiFi hotspot with SSID and password.

## 2. State Query

### 2.1 Basic Information

| Method | Return Type | Description |
|--------|-------------|-------------|
| GetPower() | uint32_t | Battery percentage |
| GetTemperature() | float32_t | Device temperature (°C) |
| GetSn() | string | Serial number |
| GetDevName() | string | Device name |
| GetWifiSsid() | string | WiFi SSID |
| GetModel() | Model | Robot model type |

### 2.2 Speed & Mode

| Method | Return Type | Description |
|--------|-------------|-------------|
| GetSpeed() | SpeedInfo | Speed information |
| GetSpeedLevel() | SpeedLevel | Current speed level |
| GetFunctionMode() | FunctionMode | Current function mode |
| GetControlMode() | ControlMode | Current control mode |
| GetMotionMode() | MotionMode | Motion mode status |
| GetMotionType() | MotionType | Current motion type |
| GetVersion() | VersionInfo | Firmware versions |

### 2.3 IMU Data

| Method | Return Type | Description |
|--------|-------------|-------------|
| GetQuaternion() | array<float32_t, 4> | IMU quaternion [w, x, y, z] |
| GetRPY() | array<float32_t, 3> | Euler angles [Roll, Pitch, Yaw] (rad) |
| GetBodyAcc() | array<float32_t, 3> | Body frame acceleration (m/s²) |
| GetBodyGyro() | array<float32_t, 3> | Body frame angular velocity (rad/s) |

### 2.4 Odometry Data

| Method | Return Type | Description |
|--------|-------------|-------------|
| GetPosition() | array<float32_t, 3> | World position [x, y, z] (m) |
| GetWorldVelocity() | array<float32_t, 3> | World frame velocity [x, y, z] (m/s) |
| GetBodyVelocity() | array<float32_t, 3> | Body frame velocity [x, y, z] (m/s) |

### 2.5 Motor & Joint Data

| Method | Return Type | Description |
|--------|-------------|-------------|
| GetMotorTemp() | array<float32_t, 16> | Motor temperatures |
| GetLegAbadJoint() | array<float32_t, 4> | Abduction joint angles (rad) |
| GetLegHipJoint() | array<float32_t, 4> | Hip joint angles (rad) |
| GetLegKneeJoint() | array<float32_t, 4> | Knee joint angles (rad) |
| GetLegFootJoint() | array<float32_t, 4> | Foot joint angles (rad) |
| GetLegAbadJointVel() | array<float32_t, 4> | Abduction joint velocities (rad/s) |
| GetLegHipJointVel() | array<float32_t, 4> | Hip joint velocities (rad/s) |
| GetLegKneeJointVel() | array<float32_t, 4> | Knee joint velocities (rad/s) |
| GetLegFootJointVel() | array<float32_t, 4> | Foot joint velocities (rad/s) |
| GetLegAbadJointTorque() | array<float32_t, 4> | Abduction joint torques (N·m) |
| GetLegHipJointTorque() | array<float32_t, 4> | Hip joint torques (N·m) |
| GetLegKneeJointTorque() | array<float32_t, 4> | Knee joint torques (N·m) |
| GetLegFootJointTorque() | array<float32_t, 4> | Foot joint torques (N·m) |

> Joint arrays follow the order: [Left-Front, Right-Front, Left-Rear, Right-Rear]

### 2.6 Fault & Battery

| Method | Return Type | Description |
|--------|-------------|-------------|
| GetFaultInfo() | vector<FaultInfo> | Fault information list |
| GetBatteryInfo() | BatteryInfo | Battery details |

### 2.7 WiFi Configuration

| Method | Return Type | Description |
|--------|-------------|-------------|
| GetWifiResutlf() | optional<bool> | WiFi config result (true=success, false=failure, nullopt=no result) |
| IsRecvWifiResult() | bool | Whether WiFi config result has been received |

---

## 3. Data Types

### 3.1 CmdCode

| Value | Hex | Description |
|-------|-----|-------------|
| CMD_NULL | 0x00 | No operation |
| CMD_EMERGENCY_STOP | 0x5A | Emergency stop |
| CMD_STAND_UP | 0x7A | Stand up |
| CMD_SIT_DOWN | 0x6A | Sit down |
| CMD_MOVE_MODE | 0x8A | Move mode |
| CMD_BALANCE_STAND_MODE | 0x9A | Balance stand mode |
| CMD_LOCK_MODE | 0x9B | Lock mode |
| CMD_SLOW_SPEED | 0xAE | Slow speed |
| CMD_NORMAL_SPEED | 0xAF | Normal speed |
| CMD_FAST_SPEED | 0xB0 | Fast speed |
| CMD_JUMP | 0x01 | Jump |
| CMD_FORWARD_JUMP | 0x02 | Forward jump |
| CMD_BACK_FLIP | 0x03 | Back flip |
| CMD_GREET | 0x04 | Greet |
| CMD_TWO_LEG_STAND | 0x05 | Two leg stand |
| CMD_CRAWL_FORWARD | 0x09 | Crawl forward |
| CMD_CLIMBING_HIGH_PLATFORM | 0x0A | Climb high platform |
| CMD_HAND_STAND | 0x0B | Hand stand |
| CMD_UNLOAD_SQUAT | 0x0C | Unload squat |
| CMD_ENTER_LAB_MODE | 0xC1 | Enter lab mode |
| CMD_EXIT_LAB_MODE | 0xC2 | Exit lab mode |
| CMD_ENTER_ENTERTAINMENT_MODE | 0xC3 | Enter entertainment mode |
| CMD_EXIT_ENTERTAINMENT_MODE | 0xC4 | Exit entertainment mode |
| CMD_REMOTE_CONTROL_RIGHT | 0xAA | Request remote control |
| CMD_SDK_CONTROL_RIGHT | 0xB3 | Request SDK control |
| CMD_ROAMERX_CONTROL_RIGHT | 0xB4 | Request roamer control |
| CMD_GENERAL_SDK_CONTROL_RIGHT | 0xB5 | Request general SDK control |

### 3.2 Structs

#### SpeedInfo
| Field | Type | Description |
|-------|------|-------------|
| speed | float32_t | Forward/backward speed |
| angle_speed | float32_t | Angular velocity |
| shift_speed | float32_t | Lateral speed |
| angle | float32_t | Head angle |

#### VersionInfo
| Field | Type | Description |
|-------|------|-------------|
| mc_version | string | Main controller version |
| dog_task_version | string | Dog task version |

#### FaultInfo
| Field | Type | Description |
|-------|------|-------------|
| module | string | Fault module |
| submodule | string | Fault submodule |
| error_code | uint32_t | Error code |
| level | string | Error level |
| info | string | Error description |

#### WifiInfo
| Field | Type | Description |
|-------|------|-------------|
| ssid | string | WiFi SSID |
| password | string | WiFi password |

#### BatteryInfo
| Field | Type | Description |
|-------|------|-------------|
| current | float32_t | Current (A) |
| error | int32_t | Error code |
| power | int32_t | Battery percentage |
| temp | float32_t | Temperature (°C) |
| volt | float32_t | Voltage (V) |

### 3.3 Enums

#### SpeedLevel
| Value | Description |
|-------|-------------|
| SL_NULL | Invalid |
| SL_SLOW | Slow |
| SL_NORMAL | Normal |
| SL_FAST | Fast |

#### Model
| Value | Description |
|-------|-------------|
| MODEL_XG | Point-foot |
| MODEL_XGW | Wheel-foot |
| MODEL_XGWHSPD | High-speed wheel-foot |

#### FunctionMode
| Value | Description |
|-------|-------------|
| FM_NULL | Invalid |
| FM_REMOTE | Remote control mode |
| FM_TRACE | Trace mode |
| FM_SDK | SDK mode |
| FM_GENERAL_SDK | General SDK mode |
| FM_ROAMER | Roamer mode |

#### ControlMode
| Value | Description |
|-------|-------------|
| CM_NULL | Invalid |
| CM_STAND_UP | Standing |
| CM_SIT_DOWN | Sitting |
| CM_LOCK_MODE | Lock mode |
| CM_EMERGENCY_STOP | Emergency stop |
| CM_MOVE_MODE | Move mode |
| CM_BALANCE_STAND_MODE | Balance stand mode |
| CM_INIT | Initializing |

#### MotionMode
| Value | Description |
|-------|-------------|
| MM_NULL | Invalid |
| MM_RUNNING | Performing trick |
| MM_REST | Not performing trick |
| MM_FORBID | Forbidden (not in lab mode) |

#### MotionType
| Value | Description |
|-------|-------------|
| MT_NULL | Invalid |
| MT_JUMP | Jump |
| MT_FORWARD_JUMP | Forward jump |
| MT_BACK_FLIP | Back flip |
| MT_GREET | Greet |
| MT_TWO_LEG_STAND | Two leg stand |
| MT_FORBID | Forbidden (rage mode not enabled) |
| MT_UNKNOWN | Unknown (not performing trick) |
| MT_UNLOADING_SQUAT | Unload squat |

#### Role
| Value | Description |
|-------|-------------|
| ROLE_NULL | Invalid |
| ROLE_REMOTE | Remote controller user |
| ROLE_SDK | SDK programmatic user |

---

See [Chinese API Reference](../zh/api.md) for the complete Chinese documentation.
