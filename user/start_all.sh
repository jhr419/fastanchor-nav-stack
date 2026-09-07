#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/lib/common.sh"

robot="$NAV_DEFAULT_ROBOT"
with_rviz=false
dry_run=false
navigation_args=()

while (($# > 0)); do
  case "$1" in
    --robot)
      (($# >= 2)) || die "--robot 缺少参数"
      robot="$2"
      shift 2
      ;;
    --rviz)
      with_rviz=true
      shift
      ;;
    --no-rviz)
      with_rviz=false
      shift
      ;;
    --dry-run)
      dry_run=true
      shift
      ;;
    --map|--lidar-model|--lio-backend|--local-target-distance)
      (($# >= 2)) || die "$1 缺少参数"
      navigation_args+=("$1" "$2")
      shift 2
      ;;
    --)
      shift
      navigation_args+=(-- "$@")
      break
      ;;
    *:=*)
      navigation_args+=("$1")
      shift
      ;;
    *) die "未知参数: $1" ;;
  esac
done

validate_robot "$robot"
parse_navigation_options "${navigation_args[@]}"

if [[ "$dry_run" == "false" ]]; then
  source_workspace
  require_command gnome-terminal
  [[ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]] || die "未检测到图形会话，无法启动 gnome-terminal"
fi

module_titles=()
module_commands=()

add_module()
{
  module_titles+=("$1")
  shift
  module_commands+=("$(shell_join "$@")")
}

common_args=(
  --map "$NAV_MAP"
  --lidar-model "$NAV_LIDAR_MODEL"
  --lio-backend "$NAV_LIO_BACKEND"
  --local-target-distance "$NAV_LOCAL_TARGET_DISTANCE"
)

add_module "Livox $NAV_LIDAR_MODEL" \
  "$SCRIPT_DIR/start_sensor.sh" --lidar-model "$NAV_LIDAR_MODEL"
add_module "LIO 与 FastAnchor 定位" \
  "$SCRIPT_DIR/start_localization.sh" "${common_args[@]}"
add_module "全局规划、SCAN 与控制" \
  "$SCRIPT_DIR/start_planner.sh" "${common_args[@]}" "${NAV_EXTRA_ARGS[@]}"

if [[ "$robot" == "zs" ]]; then
  add_module "智身 L1 速度桥" "$SCRIPT_DIR/start_zs_bridge.sh"
elif [[ "$robot" == "go2" ]]; then
  add_module "Unitree Go2 速度桥" "$SCRIPT_DIR/start_go2_bridge.sh"
fi

if [[ "$with_rviz" == "true" ]]; then
  add_module "导航 RViz" "$SCRIPT_DIR/start_rviz.sh"
fi

total=${#module_titles[@]}
echo "============================================================"
echo "FastAnchor 导航系统"
echo "============================================================"
echo "工作空间: $PROJECT_WS"
echo "地图: $NAV_MAP"
echo "雷达: $NAV_LIDAR_MODEL"
echo "LIO 后端: $NAV_LIO_BACKEND"
echo "底盘: $robot"
echo "终端数量: $total"
echo "============================================================"

terminal_args=()
for ((index = 0; index < total; ++index)); do
  number=$(printf '%02d' "$((index + 1))")
  title="$number ${module_titles[index]}"
  command="env NAV_STAGE_INDEX=$number NAV_STAGE_TOTAL=$(printf '%02d' "$total") ${module_commands[index]}"
  payload="cd $(printf '%q' "$PROJECT_WS"); $command; result=\$?; echo; echo '[$title] 已退出，状态码:' \$result; exec bash"
  terminal_command="bash -lc $(printf '%q' "$payload")"

  if [[ "$dry_run" == "true" ]]; then
    echo "$title"
    echo "  $command"
    continue
  fi

  if ((index == 0)); then
    terminal_args+=(--window --title="$title" --working-directory="$PROJECT_WS")
  else
    terminal_args+=(--tab --title="$title" --working-directory="$PROJECT_WS")
  fi
  terminal_args+=(--command="$terminal_command")
done

if [[ "$dry_run" == "true" ]]; then
  echo "Dry-run 完成，未打开终端、未启动 ROS 节点。"
  exit 0
fi

gnome-terminal "${terminal_args[@]}"
echo "完整系统启动命令已发送到 $total 个终端标签页。"
