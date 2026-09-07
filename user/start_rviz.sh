#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/lib/common.sh"

config_file="src/SCAN-Planner/planner/plan_manage/launch/default.rviz"
if (($# > 0)); then
  [[ "$1" == "--config" && $# -eq 2 ]] || die "用法: $0 [--config RViz配置]"
  config_file="$2"
fi
config_file="$(resolve_workspace_path "$config_file")"
require_file "$config_file" "RViz 配置文件"

source_workspace
banner "${NAV_STAGE_INDEX:-01}" "${NAV_STAGE_TOTAL:-01}" "导航 RViz"
wait_for_topic "/fast_anchor/odom" "FastAnchor 定位"
echo "启动 RViz: $config_file"
exec_managed rviz ros2 run rviz2 rviz2 -d "$config_file" -f map
