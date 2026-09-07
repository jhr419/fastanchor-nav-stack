#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/lib/common.sh"
source_workspace

output="bags/navigation_$(date +%Y%m%d_%H%M%S)"
if (($# > 0)); then
  [[ "$1" == "--output" && $# -eq 2 ]] || die "用法: $0 [--output 录包目录]"
  output="$2"
fi
output="$(resolve_workspace_path "$output")"
mkdir -p "$(dirname "$output")"

banner "录包" "工程" "记录导航核心 Topic"
wait_for_topic "/fast_anchor/odom" "FastAnchor 定位"
echo "输出目录: $output"

exec_managed record ros2 bag record \
  -o "$output" \
  /livox/lidar \
  /livox/imu \
  /fast_anchor/odom \
  /fast_anchor/aligned_cloud \
  /planned_path \
  /planning/bspline \
  /cmd_vel \
  /tf \
  /tf_static
