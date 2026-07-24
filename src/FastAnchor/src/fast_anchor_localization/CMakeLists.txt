cmake_minimum_required(VERSION 3.8)
project(fast_anchor_localization)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(fast_anchor_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters registration)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

set(dependencies
  fast_anchor_interfaces
  geometry_msgs
  nav_msgs
  pcl_conversions
  pcl_ros
  rclcpp
  sensor_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

add_executable(fast_anchor_localization_node
  src/fast_anchor_localization.cpp
  src/fast_anchor_localization_node.cpp
)

target_include_directories(fast_anchor_localization_node
  PRIVATE
    include
    ${PCL_INCLUDE_DIRS}
)
ament_target_dependencies(fast_anchor_localization_node
  ${dependencies}
)
target_link_libraries(fast_anchor_localization_node
  Eigen3::Eigen
  ${PCL_LIBRARIES}
)

install(TARGETS
  fast_anchor_localization_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

install(DIRECTORY
  launch
  config
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(${dependencies})
ament_package()
