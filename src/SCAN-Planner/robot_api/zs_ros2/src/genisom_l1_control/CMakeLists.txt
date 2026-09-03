cmake_minimum_required(VERSION 3.10)
project(genisom_l1_control)

# 官方 SDK 的根 CMake 明确要求 C++20。
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Threads REQUIRED)

set(GENISOM_SDK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/genisom_L1_sdk"
  CACHE PATH "genisom_L1_sdk 根目录")
set(GENISOM_HIGHLEVEL_SDK_ROOT
  "${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/genisom_l1_sdk_old"
  CACHE PATH "旧版 genisom_l1_sdk_old 根目录")

if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64|ARM64")
  set(GENISOM_SDK_ARCH "aarch64")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|amd64|AMD64")
  set(GENISOM_SDK_ARCH "x86_64")
else()
  message(FATAL_ERROR "官方 GENISOM SDK 不支持当前架构: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

set(GENISOM_SDK_LIBRARY
  "${GENISOM_SDK_ROOT}/lib/${GENISOM_SDK_ARCH}/libzsibot.a")
if(NOT EXISTS "${GENISOM_SDK_ROOT}/include/zsibot_sdk/zsibot_api.h")
  message(FATAL_ERROR "未找到 GENISOM SDK 头文件: ${GENISOM_SDK_ROOT}")
endif()
if(NOT EXISTS "${GENISOM_SDK_LIBRARY}")
  message(FATAL_ERROR "未找到 GENISOM SDK 静态库: ${GENISOM_SDK_LIBRARY}")
endif()
set(GENISOM_HIGHLEVEL_LIBRARY_DIR
  "${GENISOM_HIGHLEVEL_SDK_ROOT}/lib/zsl-1w/${GENISOM_SDK_ARCH}")
set(GENISOM_HIGHLEVEL_LIBRARY
  "${GENISOM_HIGHLEVEL_LIBRARY_DIR}/libmc_sdk_zsl_1w_${GENISOM_SDK_ARCH}.so")
if(NOT EXISTS "${GENISOM_HIGHLEVEL_SDK_ROOT}/include/zsl-1w/highlevel.h")
  message(FATAL_ERROR "未找到旧版 GENISOM highlevel 头文件: ${GENISOM_HIGHLEVEL_SDK_ROOT}")
endif()
if(NOT EXISTS "${GENISOM_HIGHLEVEL_LIBRARY}")
  message(FATAL_ERROR "未找到旧版 GENISOM highlevel 动态库: ${GENISOM_HIGHLEVEL_LIBRARY}")
endif()

add_library(genisom_sdk STATIC IMPORTED GLOBAL)
set_target_properties(genisom_sdk PROPERTIES
  IMPORTED_LOCATION "${GENISOM_SDK_LIBRARY}"
  INTERFACE_INCLUDE_DIRECTORIES "${GENISOM_SDK_ROOT}/include")

add_library(genisom_highlevel_sdk SHARED IMPORTED GLOBAL)
set_target_properties(genisom_highlevel_sdk PROPERTIES
  IMPORTED_LOCATION "${GENISOM_HIGHLEVEL_LIBRARY}"
  INTERFACE_INCLUDE_DIRECTORIES "${GENISOM_HIGHLEVEL_SDK_ROOT}/include")

add_library(genisom_sdk_wrapper src/sdk_wrapper.cpp)
target_include_directories(genisom_sdk_wrapper PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(genisom_sdk_wrapper PUBLIC genisom_sdk Threads::Threads)

add_library(genisom_highlevel_velocity_client STATIC
  src/highlevel_velocity_client.cpp)
target_include_directories(genisom_highlevel_velocity_client PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(genisom_highlevel_velocity_client PUBLIC genisom_highlevel_sdk)

add_library(genisom_process_supervisor src/process_supervisor.cpp)
target_include_directories(genisom_process_supervisor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(genisom_manager_node
  src/manager_node.cpp
  src/main.cpp)
target_link_libraries(genisom_manager_node
  genisom_sdk_wrapper
  genisom_process_supervisor)
ament_target_dependencies(genisom_manager_node
  diagnostic_msgs
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs)

add_executable(genisom_twist_node
  src/twist_node.cpp
  src/twist_main.cpp)
target_link_libraries(genisom_twist_node
  genisom_highlevel_velocity_client
  genisom_process_supervisor)
ament_target_dependencies(genisom_twist_node
  diagnostic_msgs
  geometry_msgs
  rclcpp
  std_msgs)

install(TARGETS
  genisom_manager_node
  genisom_twist_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(FILES
  "${GENISOM_HIGHLEVEL_LIBRARY}"
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

set_target_properties(genisom_twist_node PROPERTIES
  INSTALL_RPATH "$ORIGIN")

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_command_mapping test/test_command_mapping.cpp)
  target_link_libraries(test_command_mapping
    genisom_sdk_wrapper
    genisom_highlevel_velocity_client)

  find_package(ament_cmake_cpplint REQUIRED)
  ament_cpplint(FILTERS
    "-legal/copyright"
    "-readability/namespace"
    "-build/header_guard")

  # 各检查项显式注册，避免自动 cpplint 覆盖中文注释过滤规则。
  find_package(ament_cmake_cppcheck REQUIRED)
  ament_cppcheck()
  find_package(ament_cmake_flake8 REQUIRED)
  ament_flake8()
  find_package(ament_cmake_lint_cmake REQUIRED)
  ament_lint_cmake()
  find_package(ament_cmake_pep257 REQUIRED)
  ament_pep257()
  find_package(ament_cmake_uncrustify REQUIRED)
  ament_uncrustify()
  find_package(ament_cmake_xmllint REQUIRED)
  ament_xmllint()
endif()

ament_package()
