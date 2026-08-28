#!/bin/bash

source /opt/ros/humble/setup.bash || return 1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_LOCALHOST_ONLY=0

unitree_network_interface="${UNITREE_NETWORK_INTERFACE:-}"
if [[ -z "${unitree_network_interface}" ]] && command -v ip >/dev/null 2>&1; then
  # 优先选择 Unitree 默认网段上的活动网卡，避免误选 Wi-Fi 或虚拟网卡。
  unitree_network_interface="$(
    ip -o -4 addr show up scope global 2>/dev/null |
      awk '$4 ~ /^192\.168\.123\./ {print $2; exit}'
  )"
fi

if [[ -n "${unitree_network_interface}" ]]; then
  if [[ ! "${unitree_network_interface}" =~ ^[a-zA-Z0-9_.:-]+$ ]]; then
    echo "错误：UNITREE_NETWORK_INTERFACE 包含非法字符。" >&2
    return 1
  fi
  export UNITREE_NETWORK_INTERFACE="${unitree_network_interface}"
  export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"${unitree_network_interface}\" priority=\"default\" multicast=\"default\" /></Interfaces></General></Domain></CycloneDDS>"
  echo "Unitree ROS 2 环境：CycloneDDS，网卡 ${unitree_network_interface}"
else
  unset CYCLONEDDS_URI
  echo "警告：未找到 192.168.123.x 网卡，CycloneDDS 将自动选择网卡。" >&2
fi

unset unitree_network_interface
