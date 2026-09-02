# Zsibot SDK Communication Protocol

## Overview

The Zsibot SDK communicates with the robot dog via **UDP** protocol using **JSON** serialization. The architecture uses three independent threads for sending commands, receiving data, and heartbeat maintenance, ensuring reliable and low-latency communication.

## Network Configuration

| Parameter | Default Value | Description |
|-----------|---------------|-------------|
| Robot IP  | 192.168.234.1 | Robot's IP address |
| Send Port | 8081          | Port for sending commands to robot |
| Recv Port | 8080          | Port for receiving data from robot |

## Message Format

All messages are JSON strings sent over UDP.

### Send Messages (SDK → Robot)

#### Heartbeat
```json
{"type": "heartbeat", "role": "remote"}
```
Sent every 50ms to maintain connection. Timeout: 5 seconds.

#### Command
```json
{"type": "cmd", "role": "remote", "cmd": 122}
```
`cmd` is the integer value of the CmdCode enum.

#### Remote Control
```json
{
  "type": "remote",
  "role": "remote",
  "joystick": [0.5, 0, 0, 0],
  "button": [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
}
```
- `joystick`: 4 float values — [forward/backward, rotation, left/right, head_angle]
- `button`: 14 float values — button states

Joystick values with absolute value > 1 are automatically normalized to [-1, 1].

#### WiFi Configuration
```json
{"type": "wifi_hotspot_rename", "role": "remote", "name": "new_ssid", "password": "new_password"}
```

### Receive Messages (Robot → SDK)

| Type | Description |
|------|-------------|
| dog_state | Basic robot state (power, temperature, speed, device name, WiFi SSID, serial number, model) |
| version | Firmware version information (MC version, dog_task version) |
| speed_set | Speed level configuration |
| feedback | Current modes (function mode, control mode, motion mode, motion type) and motor temperatures |
| fault_info | Fault/error information (module, submodule, error code, level, info) |
| imu_info | IMU data (quaternion, gyro, acceleration, RPY) |
| odom_info | Odometry data (world velocity, position, world angular velocity, body velocity, body angular velocity) |
| leg_joint_info | Leg joint angles, velocities, and torques (4 legs × 4 joints) |
| dev_info | Device info including battery status (voltage, current, temperature, power, error code) |
| wifi_hotspot_rename | WiFi configuration result |

#### Leg Joint Data Order
Joint arrays follow the order: `[Left-Front, Right-Front, Left-Rear, Right-Rear]`

Each leg has 4 joints: Abduction, Hip, Knee, Foot

## Command Codes (CmdCode)

| Code | Hex Value | Description |
|------|-----------|-------------|
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

> **Note:** Jump, forward jump, back flip, greet, two leg stand, and unload squat require Lab Mode (`CMD_ENTER_LAB_MODE`) to be active first.
> Wheel-foot models cannot perform jump, forward jump, greet, back flip, or two leg stand actions.

## Roles

| Role | Description |
|------|-------------|
| remote | Remote controller user (default) |
| sdk | SDK programmatic control |

SDK users must first request control rights via `CMD_SDK_CONTROL_RIGHT` before sending movement commands. Control switching is based on feedback confirmation, not immediate trigger.

## Supported Models

| Model | Description | Restrictions |
|-------|-------------|-------------|
| MODEL_XG | Point-foot | Cannot crawl, climb, or hand stand |
| MODEL_XGW | Wheel-foot | Cannot jump, flip, greet, or two-leg stand |
| MODEL_XGWHSPD | High-speed wheel-foot | Same as wheel-foot |

## Connection Management

- **Heartbeat interval:** 50ms
- **Connection timeout:** 5 seconds
- **Connection condition:** At least one data packet received and last heartbeat within 5 seconds
- **`IsConnected()`** returns `true` when both conditions are met
