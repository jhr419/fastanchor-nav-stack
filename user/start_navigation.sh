#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/lib/common.sh"
parse_navigation_options "$@"
source_workspace

banner "${NAV_STAGE_INDEX:-01}" "${NAV_STAGE_TOTAL:-01}" "一体化导航系统"
echo "地图: $NAV_MAP_ABS"
echo "雷达: $NAV_LIDAR_MODEL"
echo "LIO 后端: $NAV_LIO_BACKEND"
echo "局部目标距离: $NAV_LOCAL_TARGET_DISTANCE m"

exec_managed navigation \
  ros2 launch navigation_bringup navigation_system.launch.py \
  map_pcd_path:="$NAV_MAP_ABS" \
  lidar_model:="$NAV_LIDAR_MODEL" \
  lio_backend:="$NAV_LIO_BACKEND" \
  local_target_distance:="$NAV_LOCAL_TARGET_DISTANCE" \
  start_localization_rviz:=false \
  start_global_planner_rviz:=false \
  start_local_planner_rviz:=false \
  "${NAV_EXTRA_ARGS[@]}"
