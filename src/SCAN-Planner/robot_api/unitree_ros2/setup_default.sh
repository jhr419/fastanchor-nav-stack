#!/bin/bash

source /opt/ros/humble/setup.bash || return 1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_LOCALHOST_ONLY=0

if [[ -n "${UNITREE_NETWORK_INTERFACE:-}" ]]; then
  if [[ ! "${UNITREE_NETWORK_INTERFACE}" =~ ^[a-zA-Z0-9_.:-]+$ ]]; then
    echo "错误：UNITREE_NETWORK_INTERFACE 包含非法字符。" >&2
    return 1
  fi
  export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"${UNITREE_NETWORK_INTERFACE}\" priority=\"default\" multicast=\"default\" /></Interfaces></General></Domain></CycloneDDS>"
  echo "Unitree ROS 2 环境：CycloneDDS，网卡 ${UNITREE_NETWORK_INTERFACE}"
else
  unset CYCLONEDDS_URI
  echo "Unitree ROS 2 环境：CycloneDDS，自动选择网卡"
fi
