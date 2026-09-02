# Zsibot SDK

Zsibot Quadruped Robot SDK, developed in C++20, provides a high-level API over UDP protocol for sending control commands and querying real-time sensor status of Zsibot robot dogs.

## Features

- Remote control via JSON over UDP
- SDK programmatic control (role-based control arbitration)
- Roamer mode control (RoamerX)
- General SDK mode control
- Real-time sensor data reading (IMU, odometry, joint states, battery)
- Connection monitoring (heartbeat timeout detection)
- Multi-model support (point-foot / wheel-foot / high-speed wheel-foot)
- WiFi hotspot configuration
- Fault information query
- Entertainment / Lock / Balance stand modes
- Unload squat function
- Three-thread async architecture (send, receive, heartbeat fully decoupled)
- Pimpl pattern for ABI stability

## Directory Structure

```
├── CMakeLists.txt          # Build configuration
├── docs/                   # Documentation
│   ├── en/                 # English docs
│   ├── zh/                 # Chinese docs
│   └── protocol/           # Communication protocol docs
├── example/                # Example programs
│   ├── CMakeLists.txt
│   ├── remote_control.cpp  # Remote control example
│   └── sdk_control.cpp     # SDK control example
├── include/
│   └── zsibot_sdk/         # Public headers
│       ├── zsibot_api.h        # Main class declaration
│       ├── zsibot_define.h     # Data type definitions
│       └── zsibot_exception.h  # Custom exceptions
├── lib/                    # Prebuilt static libraries
│   ├── aarch64/
│   │   └── libzsibot.a
│   └── x86_64/
│       └── libzsibot.a
└── CHANGELOG.md            # Changelog
```

## Quick Start

### Prerequisites

- CMake 3.10+
- C++20 compiler (GCC 10+ / Clang 12+)
- libnetwork.a (UDP networking library, already merged into prebuilt library)

### Build from Source

```bash
mkdir build && cd build
cmake ..
make -j$(nproc)
```

### Build Examples

```bash
# Build via root project (recommended)
make remote_control   # Remote control example
make sdk_control      # SDK control example

# Or build examples independently (requires SDK installed first)
cd example && mkdir build && cd build
cmake .. && make
```

### Install

```bash
sudo make install
# Headers → /usr/local/include/zsibot_sdk/
# Static lib → /usr/local/lib/libzsibot.a
```

### Usage Example

```cpp
#include "zsibot_sdk/zsibot_api.h"

using namespace zsibot;

int main() {
    // Create executor (default: remote role)
    ZsibotExecutor exec;

    // Stand up
    exec.SetCmd(CmdCode::CMD_STAND_UP);

    // Move forward
    exec.SetRemote({0.5, 0, 0, 0}, std::array<float32_t, 14>{0});

    // Query status
    std::cout << "Battery: " << exec.GetPower() << "%" << std::endl;
    std::cout << "Temperature: " << exec.GetTemperature() << "°C" << std::endl;

    // Battery details
    BatteryInfo battery = exec.GetBatteryInfo();
    std::cout << "Voltage: " << battery.volt << " V" << std::endl;
    std::cout << "Current: " << battery.current << " A" << std::endl;

    // IMU data
    auto quat = exec.GetQuaternion();
    auto rpy = exec.GetRPY();

    // Joint data (order: left-front, right-front, left-rear, right-rear)
    auto hip_angles = exec.GetLegHipJoint();

    return 0;
}
```

### Link SDK in Your Project

```cmake
# Option 1: Using find_library
find_library(ZSIBOT_SDK_LIB zsibot PATHS /usr/local/lib)
target_include_directories(your_target PRIVATE /usr/local/include)
target_link_libraries(your_target PRIVATE ${ZSIBOT_SDK_LIB} Threads::Threads)

# Option 2: Via add_subdirectory
add_subdirectory(path/to/zsibot_sdk)
target_link_libraries(your_target PRIVATE zsibot_sdk)
```

## Supported Platforms

| Platform | Architecture | Status |
|----------|-------------|--------|
| Linux x86_64 | x86_64 | ✅ Supported |
| Linux ARM64 | aarch64 | ✅ Supported |

## Documentation

- [API Reference (English)](docs/en/api.md)
- [API Reference (中文)](docs/zh/api.md)
- [Architecture (中文)](docs/zh/architecture.md)
- [Communication Protocol](docs/protocol/protocol.md)
- [Changelog](CHANGELOG.md)

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
