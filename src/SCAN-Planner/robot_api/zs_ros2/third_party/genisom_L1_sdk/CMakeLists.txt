cmake_minimum_required(VERSION 3.10)

project(zsibot_sdk
    VERSION 1.1.0
    DESCRIPTION "Zsibot 机器狗 SDK - C++ 远程控制与状态监控开发套件"
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_BUILD_TYPE Release)

# 查找 pthread
find_package(Threads REQUIRED)

# ============================================================
# SDK 静态库（使用 lib/ 目录下的预编译库）
# ============================================================
# 自动检测目标架构
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64|ARM64")
    set(DETECTED_ARCH "aarch64")
else()
    set(DETECTED_ARCH "x86_64")
endif()
set(ZSIBOT_ARCH "${DETECTED_ARCH}" CACHE STRING "目标架构: x86_64 或 aarch64 (自动检测)")
set_property(CACHE ZSIBOT_ARCH PROPERTY STRINGS x86_64 aarch64)

set(PREBUILT_LIB "${PROJECT_SOURCE_DIR}/lib/${ZSIBOT_ARCH}/libzsibot.a")
if(NOT EXISTS "${PREBUILT_LIB}")
    message(FATAL_ERROR
        "未找到预编译库: ${PREBUILT_LIB}\n"
        "请确认 lib/${ZSIBOT_ARCH}/ 目录下存在 libzsibot.a")
endif()
message(STATUS "使用预编译库: ${PREBUILT_LIB}")

add_library(${PROJECT_NAME} STATIC IMPORTED GLOBAL)
set_target_properties(${PROJECT_NAME} PROPERTIES
    IMPORTED_LOCATION "${PREBUILT_LIB}"
)

target_include_directories(${PROJECT_NAME}
    INTERFACE
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} INTERFACE Threads::Threads)

# ============================================================
# 示例程序
# ============================================================
add_subdirectory(example)

# ============================================================
# 安装
# ============================================================
# 安装公共头文件到 /usr/local/include/zsibot_sdk/
install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/zsibot_sdk
        DESTINATION include)

# 安装预编译静态库到 /usr/local/lib/
install(FILES "${PREBUILT_LIB}"
        DESTINATION lib
        RENAME libzsibot.a)

# ============================================================
# CPack 打包配置（生成 deb 包）
# ============================================================
set(CPACK_PACKAGING_INSTALL_PREFIX "/usr/local/")
set(CPACK_GENERATOR "DEB")
set(CPACK_PACKAGE_CONTACT "zsibot <support@zsibot.com>")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "zsibot")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Zsibot 机器狗 SDK 开发套件")
include(CPack)
