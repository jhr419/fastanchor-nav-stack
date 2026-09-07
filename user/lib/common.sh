#!/usr/bin/env bash

COMMON_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
USER_DIR="$(cd "$COMMON_DIR/.." && pwd)"
PROJECT_WS="$(cd "$USER_DIR/.." && pwd)"
RUN_DIR="$USER_DIR/.run"

USER_CONFIG_FILE="${NAV_USER_CONFIG:-$USER_DIR/config.env}"
if [[ -f "$USER_CONFIG_FILE" ]]; then
  source "$USER_CONFIG_FILE"
fi

NAV_ROS_DISTRO="${NAV_ROS_DISTRO:-humble}"
NAV_DEFAULT_MAP="${NAV_DEFAULT_MAP:-maps/map_preprocessed2.pcd}"
NAV_DEFAULT_LIDAR_MODEL="${NAV_DEFAULT_LIDAR_MODEL:-mid360s}"
NAV_DEFAULT_LIO_BACKEND="${NAV_DEFAULT_LIO_BACKEND:-fastlio2}"
NAV_DEFAULT_LOCAL_TARGET_DISTANCE="${NAV_DEFAULT_LOCAL_TARGET_DISTANCE:-4.0}"
NAV_DEFAULT_ROBOT="${NAV_DEFAULT_ROBOT:-zs}"
NAV_TOPIC_WAIT_TIMEOUT="${NAV_TOPIC_WAIT_TIMEOUT:-0}"

die()
{
  echo "[错误] $*" >&2
  exit 1
}

banner()
{
  local index="$1"
  local total="$2"
  local name="$3"
  echo "============================================================"
  echo "[$index/$total] $name"
  echo "============================================================"
  echo "工作空间: $PROJECT_WS"
}

resolve_workspace_path()
{
  local path="$1"
  if [[ "$path" = /* ]]; then
    printf '%s\n' "$path"
  else
    printf '%s\n' "$PROJECT_WS/$path"
  fi
}

require_file()
{
  local path="$1"
  local description="$2"
  [[ -f "$path" ]] || die "$description不存在: $path"
}

require_command()
{
  local command_name="$1"
  command -v "$command_name" >/dev/null 2>&1 || die "缺少命令: $command_name"
}

source_ros()
{
  local ros_setup="/opt/ros/$NAV_ROS_DISTRO/setup.bash"
  local restore_nounset=false
  require_file "$ros_setup" "ROS 2 环境脚本"
  [[ "$-" == *u* ]] && restore_nounset=true
  set +u
  source "$ros_setup"
  if [[ "$restore_nounset" == "true" ]]; then
    set -u
  fi
}

source_workspace()
{
  local restore_nounset=false
  source_ros
  local workspace_setup="$PROJECT_WS/install/setup.bash"
  require_file "$workspace_setup" "工作空间安装环境，请先运行 user/build.sh"
  [[ "$-" == *u* ]] && restore_nounset=true
  set +u
  source "$workspace_setup"
  if [[ "$restore_nounset" == "true" ]]; then
    set -u
  fi
  export FASTANCHOR_NAV_WS="$PROJECT_WS"
}

source_go2_environment()
{
  local restore_nounset=false
  source_workspace
  local unitree_setup="$PROJECT_WS/src/SCAN-Planner/robot_api/unitree_ros2/setup_default.sh"
  require_file "$unitree_setup" "Unitree 环境脚本"
  [[ "$-" == *u* ]] && restore_nounset=true
  set +u
  source "$unitree_setup"
  if [[ "$restore_nounset" == "true" ]]; then
    set -u
  fi
}

wait_for_topic()
{
  local topic="$1"
  local description="${2:-$topic}"
  local started_at=$SECONDS

  echo "等待 $description ($topic) ..."
  until ros2 topic list 2>/dev/null | grep -Fxq -- "$topic"; do
    if (( NAV_TOPIC_WAIT_TIMEOUT > 0 && SECONDS - started_at >= NAV_TOPIC_WAIT_TIMEOUT )); then
      die "等待 Topic 超时: $topic"
    fi
    sleep 1
  done
  echo "已发现 $description ($topic)"
}

validate_lidar_model()
{
  case "$1" in
    mid360|mid360s) ;;
    *) die "不支持的雷达型号 '$1'，可选值: mid360、mid360s" ;;
  esac
}

validate_lio_backend()
{
  case "$1" in
    fastlio2|yifanlio) ;;
    *) die "不支持的 LIO 后端 '$1'，可选值: fastlio2、yifanlio" ;;
  esac
}

validate_robot()
{
  case "$1" in
    zs|go2|none) ;;
    *) die "不支持的底盘 '$1'，可选值: zs、go2、none" ;;
  esac
}

validate_positive_number()
{
  local value="$1"
  local name="$2"
  awk -v value="$value" 'BEGIN {exit !(value ~ /^[0-9]+([.][0-9]+)?$/ && value > 0)}' ||
    die "$name必须是正数，当前值: $value"
}

parse_navigation_options()
{
  NAV_MAP="$NAV_DEFAULT_MAP"
  NAV_LIDAR_MODEL="$NAV_DEFAULT_LIDAR_MODEL"
  NAV_LIO_BACKEND="$NAV_DEFAULT_LIO_BACKEND"
  NAV_LOCAL_TARGET_DISTANCE="$NAV_DEFAULT_LOCAL_TARGET_DISTANCE"
  NAV_EXTRA_ARGS=()

  while (($# > 0)); do
    case "$1" in
      --map)
        (($# >= 2)) || die "--map 缺少路径"
        NAV_MAP="$2"
        shift 2
        ;;
      --lidar-model)
        (($# >= 2)) || die "--lidar-model 缺少参数"
        NAV_LIDAR_MODEL="$2"
        shift 2
        ;;
      --lio-backend)
        (($# >= 2)) || die "--lio-backend 缺少参数"
        NAV_LIO_BACKEND="$2"
        shift 2
        ;;
      --local-target-distance)
        (($# >= 2)) || die "--local-target-distance 缺少参数"
        NAV_LOCAL_TARGET_DISTANCE="$2"
        shift 2
        ;;
      --)
        shift
        NAV_EXTRA_ARGS=("$@")
        break
        ;;
      *:=*)
        NAV_EXTRA_ARGS+=("$1")
        shift
        ;;
      *)
        die "未知参数: $1"
        ;;
    esac
  done

  validate_lidar_model "$NAV_LIDAR_MODEL"
  validate_lio_backend "$NAV_LIO_BACKEND"
  validate_positive_number "$NAV_LOCAL_TARGET_DISTANCE" "局部目标距离"
  NAV_MAP_ABS="$(resolve_workspace_path "$NAV_MAP")"
  require_file "$NAV_MAP_ABS" "地图文件"
}

process_start_time()
{
  local pid="$1"
  [[ -r "/proc/$pid/stat" ]] || return 1
  awk '{print $22}' "/proc/$pid/stat"
}

register_process()
{
  local module="$1"
  local pid_file="$RUN_DIR/$module.pid"
  mkdir -p "$RUN_DIR"

  if [[ -f "$pid_file" ]]; then
    local old_pid old_start old_module
    read -r old_pid old_start old_module < "$pid_file" || true
    if [[ -n "${old_pid:-}" && -n "${old_start:-}" ]] &&
      [[ "$(process_start_time "$old_pid" 2>/dev/null || true)" == "$old_start" ]]
    then
      die "$old_module 已在运行，PID=$old_pid"
    fi
    rm -f "$pid_file"
  fi

  printf '%s %s %s\n' "$$" "$(process_start_time "$$")" "$module" > "$pid_file"
}

exec_managed()
{
  local module="$1"
  shift
  register_process "$module"
  exec "$@"
}

shell_join()
{
  printf '%q ' "$@"
}
