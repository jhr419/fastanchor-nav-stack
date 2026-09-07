#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/lib/common.sh"

config_file=""
if (($# > 0)); then
  [[ "$1" == "--config" && $# -eq 2 ]] || die "用法: $0 [--config 配置文件]"
  config_file="$(resolve_workspace_path "$2")"
  require_file "$config_file" "Go2 速度桥配置文件"
fi

source_go2_environment
banner "${NAV_STAGE_INDEX:-01}" "${NAV_STAGE_TOTAL:-01}" "Unitree Go2 速度桥"
wait_for_topic "/cmd_vel" "导航速度指令"
echo "启动 Unitree Go2 速度桥。请确保遥控器可随时接管或急停。"

command=(ros2 launch go2_twist_bridge twist_bridge.launch.py)
if [[ -n "$config_file" ]]; then
  command+=(config_file:="$config_file")
fi
exec_managed chassis "${command[@]}"
