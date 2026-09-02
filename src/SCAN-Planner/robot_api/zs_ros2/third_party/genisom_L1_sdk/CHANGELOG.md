# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/).

## [1.0.0] - 2026-06-22

### 控制功能

- 遥控模式控制（`SetRemote()`，摇杆 + 按钮输入，自动归一化）
- SDK 程序化控制（`SetCmd()`，指令码驱动）
- 漫游者模式控制（`CMD_ROAMERX_CONTROL_RIGHT`）
- 通用 SDK 模式控制（`CMD_GENERAL_SDK_CONTROL_RIGHT`）
- WiFi 热点配置（`SetWifi()`，SSID + 密码）

### 运动指令

- 基础运动：站立 / 趴下 / 移动模式 / 紧急停止
- 模式切换：平衡站立模式 / 锁定模式
- 速度档位：慢速 / 正常速度 / 快速
- 特技动作：跳跃 / 前跳 / 后空翻 / 招手 / 双腿站立 / 匍匐前进 / 爬高台 / 双腿倒立 / 卸货下蹲
- 模式管理：进入/退出实验室模式、进入/退出文娱模式

### 状态查询接口（共 35 个 getter）

- **基础信息**：电量、温度、序列号、设备名、WiFi SSID、机型、版本信息
- **速度与模式**：速度信息（`SpeedInfo`）、速度档位、功能模式、控制模式、运动模式、运动类型
- **IMU 数据**：四元数、欧拉角（RPY）、机体加速度、机体角速度
- **里程计数据**：世界坐标位置、世界速度、机体速度
- **关节数据**：4 腿 × 4 关节（外展/髋/膝/足）的角度、角速度、力矩（共 12 个接口）
- **电机温度**：16 路电机温度
- **故障信息**：故障模块、子模块、错误码、等级、描述
- **电池信息**：电流、电压、电量百分比、温度、错误码
- **WiFi 配置**：配置结果查询、结果接收状态
- **连接状态**：心跳超时检测（50ms 间隔，5s 超时）

### 多机型兼容

| 机型 | 标识 | 约束 |
|------|------|------|
| 点足 | `MODEL_XG` | 禁止爬行、爬高台、倒立 |
| 轮足 | `MODEL_XGW` | 禁止跳跃、后空翻、双腿站立 |
| 高速轮足 | `MODEL_XGWHSPD` | 同轮足约束 |

### 构建与部署

- CMake 构建系统，支持 `make install` 安装与 CPack DEB 打包
- 支持平台：Linux x86_64、Linux ARM64（aarch64）
- 依赖：CMake 3.10+、C++17 编译器（GCC 10+ / Clang 12+）
- 默认配置：机器人 IP `192.168.234.1`，发送端口 `8081`，接收端口 `8080`
- 提供遥控模式（`remote_control.cpp`）与 SDK 模式（`sdk_control.cpp`）示例程序
