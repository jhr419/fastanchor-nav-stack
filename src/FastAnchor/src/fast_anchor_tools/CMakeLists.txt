cmake_minimum_required(VERSION 3.8)
project(fast_anchor_tools)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/downsample_pcd.py
  scripts/crop_pcd.py
  scripts/transform_pcd.py
  scripts/evaluate_trajectory.py
  scripts/plot_icp_score.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  scripts
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
