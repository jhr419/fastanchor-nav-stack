#!/bin/bash

unset ROS_DOMAIN_ID
source ./src/SCAN-Planner/robot_api/unitree_ros2/setup_default.sh || return 1
source ./install/setup.bash || return 1

if ! ros2 pkg prefix unitree_go >/dev/null 2>&1; then
  echo "错误：当前工作区未提供 unitree_go，请先完成工作区构建。" >&2
  return 1
fi

echo "Unitree 消息类型：$(ros2 pkg prefix unitree_go)"
