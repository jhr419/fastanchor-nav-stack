cmake_minimum_required(VERSION 3.10)
project(lio LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 17)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# 保留原 ROS1 项目的编译优化配置
set(CMAKE_CXX_FLAGS "-O3 -march=native -ffast-math ${CMAKE_CXX_FLAGS}")

add_compile_definitions(EIGEN_DONT_PARALLELIZE)
set(PCL_WITH_OPENMP OFF CACHE BOOL "Disable PCL OpenMP")

# 开启OpenMP（多线程）
find_package(OpenMP REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies (ROS2)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 自定义消息: WheelInfo (轮速计), 与 ROS1 的 msg/wheel_info.msg 保持一致
# 注意: rosidl_generate_interfaces 的第一个参数是 CMake target 名(不能与可执行文件重名),
#       消息包名仍由 PROJECT_NAME 决定, 即 lio/msg/WheelInfo
set(LIO_INTERFACES lio_interfaces)
rosidl_generate_interfaces(${LIO_INTERFACES}
        "msg/WheelInfo.msg"
        DEPENDENCIES std_msgs
        LIBRARY_NAME ${PROJECT_NAME}
)

# 包含头文件目录
include_directories(
        ${EIGEN3_INCLUDE_DIR}
        ${PCL_INCLUDE_DIRS}
        include
        include/ikd-Tree
        include/YamlReader
)

# 添加可执行文件 (与 ROS1 的 add_executable 列表保持一致)
add_executable(lio
        include/YamlReader/YamlReader.cpp
        include/YamlReader/YamlReader.h
        src/main.cpp
        include/main.h
        src/TopicReceive.cpp
        include/TopicProcess.h
        src/common_lib.cpp
        include/common_lib.h
        src/ESEKF.cpp
        include/ESEKF.h
        include/ikd-Tree/ikd_Tree.h
        include/ikd-Tree/ikd_Tree.cpp
        include/SharedBuffers.h
        include/type.h
        src/LIONode_Func/Pub_Func.cpp
        src/LIONode_Func/Cbk_Func.cpp
        src/LIONode_Func/Other_Func.cpp
        src/LIONode_Func/Save_Func.cpp
        src/IMUProcess.cpp
        include/IMUProcess.h
)

# ROS2 依赖
ament_target_dependencies(lio
        rclcpp
        std_msgs
        sensor_msgs
        nav_msgs
        geometry_msgs
        tf2
        tf2_ros
        pcl_ros
        pcl_conversions
        livox_ros_driver2
)

# 链接本包生成的消息类型 (wheel_info)
rosidl_get_typesupport_target(cpp_typesupport_target ${LIO_INTERFACES} "rosidl_typesupport_cpp")
target_link_libraries(lio
        ${cpp_typesupport_target}
        Eigen3::Eigen
        ${PCL_LIBRARIES}
        yaml-cpp
        OpenMP::OpenMP_CXX
)

# 定义宏
target_compile_definitions(lio PRIVATE PACKAGE_ROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

# 安装可执行文件
install(TARGETS lio
        DESTINATION lib/${PROJECT_NAME}
)

# 安装 launch / yaml / rviz
install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY yaml
        DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY rviz
        DESTINATION share/${PROJECT_NAME}
)

ament_package()
