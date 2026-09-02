# Zsibot SDK

Zsibot 四足机器人 SDK，基于 C++20 开发，通过 UDP 协议提供高层 API，支持发送控制指令和实时查询机器人传感器状态。

## 功能特性

- 远程控制（JSON over UDP）
- SDK 程序化控制（基于角色的控制权仲裁）
- 漫游者模式控制（RoamerX）
- 通用 SDK 模式控制
- 实时传感器数据读取（IMU、里程计、关节状态、电池信息）
- 连接状态监测（心跳超时检测）
- 多机型兼容（点足 / 轮足 / 高速轮足）
- WiFi 热点配置
- 故障信息查询
- 文娱模式 / 锁定模式 / 平衡站立模式
- 卸货下蹲功能
- 三线程异步通信架构（收发心跳完全解耦）
- Pimpl 模式，ABI 稳定

## 目录结构

```
├── CMakeLists.txt          # 构建配置
├── docs/                   # 文档
│   ├── en/                 # 英文文档
│   ├── zh/                 # 中文文档
│   └── protocol/           # 通信协议文档
├── example/                # 示例程序
│   ├── CMakeLists.txt      # 示例构建配置
│   ├── remote_control.cpp  # 遥控模式示例
│   └── sdk_control.cpp     # SDK 模式示例
├── include/
│   └── zsibot_sdk/         # 公共头文件
│       ├── zsibot_api.h        # 主类声明
│       ├── zsibot_define.h     # 数据类型定义
│       └── zsibot_exception.h  # 自定义异常
├── lib/                    # 预编译静态库
│   ├── aarch64/
│   │   └── libzsibot.a
│   └── x86_64/
│       └── libzsibot.a
└── CHANGELOG.md            # 变更日志
```

## 快速开始

### 环境要求

- CMake 3.10+
- C++17 编译器（GCC 10+ / Clang 12+）
- libnetwork.a（UDP 网络库，已合并进预编译库）

### 从源码构建

```bash
mkdir build && cd build
cmake ..
make -j$(nproc)
```

### 构建示例程序

```bash
# 通过根项目构建（推荐）
make remote_control   # 遥控模式示例
make sdk_control      # SDK 模式示例

# 或者独立构建示例（需先安装 SDK）
cd example && mkdir build && cd build
cmake .. && make
```

### 安装

```bash
sudo make install
# 头文件 → /usr/local/include/zsibot_sdk/
# 静态库 → /usr/local/lib/libzsibot.a
```

### 代码示例

```cpp
#include "zsibot_sdk/zsibot_api.h"

using namespace zsibot;

int main() {
    // 创建执行器（默认遥控角色）
    ZsibotExecutor exec;

    // 站立
    exec.SetCmd(CmdCode::CMD_STAND_UP);

    // 前进
    exec.SetRemote({0.5, 0, 0, 0}, std::array<float32_t, 14>{0});

    // 查询状态
    std::cout << "电量: " << exec.GetPower() << "%" << std::endl;
    std::cout << "温度: " << exec.GetTemperature() << "°C" << std::endl;

    // 电池详细信息
    BatteryInfo battery = exec.GetBatteryInfo();
    std::cout << "电压: " << battery.volt << " V" << std::endl;
    std::cout << "电流: " << battery.current << " A" << std::endl;

    // IMU 数据
    auto quat = exec.GetQuaternion();
    auto rpy = exec.GetRPY();

    // 关节数据（顺序：左前、右前、左后、右后）
    auto hip_angles = exec.GetLegHipJoint();

    return 0;
}
```

### 在项目中链接 SDK

```cmake
# 方式一：使用 find_library
find_library(ZSIBOT_SDK_LIB zsibot PATHS /usr/local/lib)
target_include_directories(your_target PRIVATE /usr/local/include)
target_link_libraries(your_target PRIVATE ${ZSIBOT_SDK_LIB} Threads::Threads)

# 方式二：通过 add_subdirectory 引入
add_subdirectory(path/to/zsibot_sdk)
target_link_libraries(your_target PRIVATE zsibot_sdk)
```

## 支持平台

| 平台 | 架构 | 状态 |
|------|------|------|
| Linux x86_64 | x86_64 | ✅ 支持 |
| Linux ARM64 | aarch64 | ✅ 支持 |

## 文档

- [API 参考文档](docs/zh/api.md)
- [项目架构文档](docs/zh/architecture.md)
- [通信协议文档](docs/protocol/protocol.md)
- [English API Reference](docs/en/api.md)
- [变更日志](CHANGELOG.md)

## 许可证

本项目采用 MIT 许可证，详见 [LICENSE](LICENSE) 文件。
