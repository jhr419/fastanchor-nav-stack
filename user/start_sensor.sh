#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/lib/common.sh"

lidar_model="$NAV_DEFAULT_LIDAR_MODEL"
while (($# > 0)); do
  case "$1" in
    --lidar-model)
      (($# >= 2)) || die "--lidar-model 缺少参数"
      lidar_model="$2"
      shift 2
      ;;
    *) die "未知参数: $1" ;;
  esac
done

validate_lidar_model "$lidar_model"
source_workspace
banner "${NAV_STAGE_INDEX:-01}" "${NAV_STAGE_TOTAL:-01}" "Livox $lidar_model 雷达"

if [[ "$lidar_model" == "mid360" ]]; then
  launch_file="msg_MID360_launch.py"
else
  launch_file="msg_MID360s_launch.py"
fi

echo "启动雷达驱动: $launch_file"
exec_managed sensor ros2 launch livox_ros_driver2 "$launch_file"
