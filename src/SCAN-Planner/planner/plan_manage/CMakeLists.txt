cmake_minimum_required(VERSION 3.8)
project(scan_planner)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(bspline_opt REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(path_searching REQUIRED)
find_package(plan_env REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rviz_common REQUIRED)
find_package(scan_planner_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(traj_utils REQUIRED)
find_package(visualization_msgs REQUIRED)

set(scan_planner_dependencies
  ament_index_cpp bspline_opt geometry_msgs nav_msgs path_searching plan_env
  rclcpp scan_planner_msgs sensor_msgs std_msgs std_srvs tf2 tf2_geometry_msgs tf2_ros
  traj_utils visualization_msgs)

add_executable(scan_planner_node
  src/scan_planner_node.cpp
  src/scan_replan_fsm.cpp
  src/planner_manager.cpp)
target_include_directories(scan_planner_node PRIVATE include)
ament_target_dependencies(scan_planner_node ${scan_planner_dependencies})
target_link_libraries(scan_planner_node Eigen3::Eigen)

foreach(target open_loop_controller closed_loop_controller go2_kinematic_sim go2_gait_publisher)
  add_executable(${target} src/${target}.cpp)
  target_include_directories(${target} PRIVATE include)
  ament_target_dependencies(${target}
    bspline_opt geometry_msgs nav_msgs rclcpp scan_planner_msgs sensor_msgs
    std_msgs tf2 tf2_geometry_msgs tf2_ros)
  target_link_libraries(${target} Eigen3::Eigen)
endforeach()

add_executable(pose_marker_server src/pose_marker_server.cpp)
ament_target_dependencies(pose_marker_server
  geometry_msgs interactive_markers nav_msgs rclcpp std_srvs visualization_msgs)

add_library(scan_planner_rviz_tools SHARED src/pose_marker_tools.cpp)
ament_target_dependencies(scan_planner_rviz_tools
  ament_index_cpp pluginlib rclcpp rviz_common std_srvs)
target_link_libraries(scan_planner_rviz_tools Qt5::Widgets)
pluginlib_export_plugin_description_file(rviz_common rviz_plugins.xml)

install(TARGETS
  scan_planner_node open_loop_controller closed_loop_controller
  go2_kinematic_sim go2_gait_publisher pose_marker_server
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS scan_planner_rviz_tools
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME} OPTIONAL)
install(DIRECTORY icons DESTINATION share/${PROJECT_NAME})
install(FILES rviz_plugins.xml DESTINATION share/${PROJECT_NAME})
install(FILES launch/default.rviz DESTINATION share/${PROJECT_NAME}/rviz OPTIONAL)
install(PROGRAMS scripts/keypoint_recorder.py
  DESTINATION lib/${PROJECT_NAME} OPTIONAL)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  ament_add_gtest(test_motion_constraints test/test_motion_constraints.cpp)
  target_include_directories(test_motion_constraints PRIVATE include)
  add_launch_test(test/test_closed_loop_constraints.py TIMEOUT 30)
  add_launch_test(test/test_global_path_window_clear.py TIMEOUT 30)
  add_launch_test(test/test_planner_startup.py TIMEOUT 30)
  add_launch_test(test/test_waypoint_parameters.py TIMEOUT 30)
endif()


# BEGIN GLOBAL_PATH_WINDOW_NODE
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(
  global_path_window_node
  src/global_path_window_node.cpp
)

ament_target_dependencies(
  global_path_window_node
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  rcl_interfaces
)

install(
  TARGETS global_path_window_node
  DESTINATION lib/${PROJECT_NAME}
)
# END GLOBAL_PATH_WINDOW_NODE

ament_package()
