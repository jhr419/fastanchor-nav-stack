# 示例程序构建配置
#
# 使用方式一：通过根 CMakeLists.txt 构建（推荐）
#   mkdir build && cd build && cmake .. && make
#
# 使用方式二：独立构建（使用 lib/ 目录下的预编译库，自动检测架构）
#   cd example && mkdir build && cd build
#   cmake .. && make
#
# 使用方式三：独立构建（使用已安装到系统的 SDK）
#   sudo make install   # 安装 SDK 到系统
#   cd example && mkdir build && cd build
#   cmake .. -DZSIBOT_USE_PREBUILT=OFF && make

cmake_minimum_required(VERSION 3.10)

# 如果作为子目录被包含，则不重复定义 project
if(NOT TARGET zsibot_sdk)
    project(zsibot_sdk_examples LANGUAGES CXX)
    set(CMAKE_CXX_STANDARD 20)
    set(CMAKE_CXX_STANDARD_REQUIRED True)

    # ============================================================
    # 配置选项
    # ============================================================
    # 是否使用 lib/ 目录下的预编译库（默认开启）
    option(ZSIBOT_USE_PREBUILT "使用 lib/ 目录下的预编译静态库" ON)

    # 目标架构，自动检测当前编译环境
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64|ARM64")
        set(DETECTED_ARCH "aarch64")
    else()
        set(DETECTED_ARCH "x86_64")
    endif()
    set(ZSIBOT_ARCH "${DETECTED_ARCH}" CACHE STRING "目标架构: x86_64 或 aarch64 (自动检测)")
    set_property(CACHE ZSIBOT_ARCH PROPERTY STRINGS x86_64 aarch64)

    if(ZSIBOT_USE_PREBUILT)
        # 使用 lib/ 目录下的预编译库
        set(PREBUILT_LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../lib/${ZSIBOT_ARCH}")
        set(PREBUILT_LIB "${PREBUILT_LIB_DIR}/libzsibot.a")

        if(NOT EXISTS "${PREBUILT_LIB}")
            message(FATAL_ERROR
                "未找到预编译库: ${PREBUILT_LIB}\n"
                "请确认 lib/${ZSIBOT_ARCH}/ 目录下存在 libzsibot.a，\n"
                "或使用 -DZSIBOT_USE_PREBUILT=OFF 改用系统已安装的 SDK。")
        endif()

        message(STATUS "使用预编译库: ${PREBUILT_LIB}")
        set(SDK_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../include")
        set(SDK_LINK_LIB "${PREBUILT_LIB}")
    else()
        # 查找已安装到系统的 zsibot_sdk
        find_library(ZSIBOT_SDK_LIB zsibot PATHS /usr/local/lib)
        if(NOT ZSIBOT_SDK_LIB)
            message(FATAL_ERROR "未找到 libzsibot.a，请先编译安装 SDK，或使用 -DZSIBOT_USE_PREBUILT=ON")
        endif()

        set(SDK_INCLUDE_DIR /usr/local/include)
        set(SDK_LINK_LIB "${ZSIBOT_SDK_LIB}")
    endif()
else()
    # 作为子目录构建，使用父项目的 target
    set(SDK_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include)
    set(SDK_LINK_LIB zsibot_sdk)
endif()

# 查找 pthread（示例程序使用了 std::thread，需要链接 pthread）
find_package(Threads REQUIRED)

# 遥控模式示例
add_executable(remote_control remote_control.cpp)
target_include_directories(remote_control PRIVATE ${SDK_INCLUDE_DIR})
target_link_libraries(remote_control PRIVATE ${SDK_LINK_LIB} Threads::Threads)

# SDK 模式示例
add_executable(sdk_control sdk_control.cpp)
target_include_directories(sdk_control PRIVATE ${SDK_INCLUDE_DIR})
target_link_libraries(sdk_control PRIVATE ${SDK_LINK_LIB} Threads::Threads)
