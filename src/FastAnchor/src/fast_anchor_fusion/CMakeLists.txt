cmake_minimum_required(VERSION 3.8)
project(fast_anchor_fusion)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(odom_frame_adapter_node src/odom_frame_adapter_node.cpp)
target_include_directories(odom_frame_adapter_node PRIVATE include)
ament_target_dependencies(odom_frame_adapter_node nav_msgs rclcpp)
target_link_libraries(odom_frame_adapter_node Eigen3::Eigen)

install(TARGETS odom_frame_adapter_node DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/unitree_sportmode_to_odom.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_odom_frame_transform test/test_odom_frame_transform.cpp)
  target_include_directories(test_odom_frame_transform PRIVATE include)
  ament_target_dependencies(test_odom_frame_transform nav_msgs)
  target_link_libraries(test_odom_frame_transform Eigen3::Eigen)
endif()

ament_package()
