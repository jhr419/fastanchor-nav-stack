#!/usr/bin/env bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/lib/common.sh"
source_workspace

echo "ROS 2 环境已加载: $NAV_ROS_DISTRO"
echo "工程环境已加载: $PROJECT_WS/install/setup.bash"
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo "提示: 如需让环境保留在当前终端，请执行 source user/setup_env.sh"
fi
