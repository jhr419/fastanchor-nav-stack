#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/lib/common.sh"
parse_navigation_options "$@"
source_workspace

banner "${NAV_STAGE_INDEX:-01}" "${NAV_STAGE_TOTAL:-01}" "全局规划、SCAN 局部规划与控制"
wait_for_topic "/fast_anchor/odom" "FastAnchor 定位"
wait_for_topic "/fast_anchor/aligned_cloud" "FastAnchor 配准点云"
echo "启动规划控制链路，局部目标距离: $NAV_LOCAL_TARGET_DISTANCE m"

exec_managed planner \
  ros2 launch navigation_bringup navigation_system.launch.py \
  map_pcd_path:="$NAV_MAP_ABS" \
  lidar_model:="$NAV_LIDAR_MODEL" \
  lio_backend:="$NAV_LIO_BACKEND" \
  local_target_distance:="$NAV_LOCAL_TARGET_DISTANCE" \
  start_localization:=false \
  start_livox_driver:=false \
  start_fastlio:=false \
  start_localization_rviz:=false \
  start_global_planner_rviz:=false \
  start_local_planner_rviz:=false \
  "${NAV_EXTRA_ARGS[@]}"
