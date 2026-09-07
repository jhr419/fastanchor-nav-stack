#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/lib/common.sh"
parse_navigation_options "$@"
source_workspace

banner "${NAV_STAGE_INDEX:-01}" "${NAV_STAGE_TOTAL:-01}" "LIO 与 FastAnchor 定位"
wait_for_topic "/livox/lidar" "Livox 点云"
wait_for_topic "/livox/imu" "Livox IMU"
echo "启动定位，LIO 后端: $NAV_LIO_BACKEND"

exec_managed localization \
  ros2 launch fast_anchor_bringup fast_anchor_mid360.launch.py \
  map_pcd_path:="$NAV_MAP_ABS" \
  visualization_map_pcd_path:="$NAV_MAP_ABS" \
  icp_map_pcd_path:="$NAV_MAP_ABS" \
  lidar_model:="$NAV_LIDAR_MODEL" \
  lio_backend:="$NAV_LIO_BACKEND" \
  start_livox_driver:=false \
  start_fastlio:=true \
  start_fastlio_rviz:=false \
  rviz:=false \
  "${NAV_EXTRA_ARGS[@]}"
