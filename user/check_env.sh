#!/usr/bin/env bash
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/lib/common.sh"

robot="$NAV_DEFAULT_ROBOT"
if (($# > 0)); then
  [[ "$1" == "--robot" && $# -eq 2 ]] || die "用法: $0 [--robot zs|go2|none]"
  robot="$2"
fi
validate_robot "$robot"

banner "环境" "检查" "工程运行条件"
failures=0

check_command()
{
  local command_name="$1"
  if command -v "$command_name" >/dev/null 2>&1; then
    echo "[正常] 命令: $command_name"
  else
    echo "[缺失] 命令: $command_name"
    failures=$((failures + 1))
  fi
}

check_optional_command()
{
  local command_name="$1"
  if command -v "$command_name" >/dev/null 2>&1; then
    echo "[正常] 可选命令: $command_name"
  else
    echo "[提示] 未安装可选命令 $command_name，多终端图形启动不可用"
  fi
}

check_file()
{
  local path="$1"
  local description="$2"
  if [[ -f "$path" ]]; then
    echo "[正常] $description: $path"
  else
    echo "[缺失] $description: $path"
    failures=$((failures + 1))
  fi
}

check_ros_package()
{
  local package_name="$1"
  if ros2 pkg prefix "$package_name" >/dev/null 2>&1; then
    echo "[正常] ROS 2 包: $package_name"
  else
    echo "[缺失] ROS 2 包: $package_name"
    failures=$((failures + 1))
  fi
}

check_command bash
check_command colcon
check_file "/opt/ros/$NAV_ROS_DISTRO/setup.bash" "ROS 2 环境"
if [[ -f "/opt/ros/$NAV_ROS_DISTRO/setup.bash" ]]; then
  source_ros
  check_command ros2
fi
check_optional_command gnome-terminal
check_file "$PROJECT_WS/install/setup.bash" "工作空间安装环境"
check_file "$(resolve_workspace_path "$NAV_DEFAULT_MAP")" "默认地图"
check_file "$PROJECT_WS/src/navigation_bringup/launch/navigation_system.launch.py" "导航 Launch"

if [[ "$robot" == "zs" ]]; then
  check_file "$PROJECT_WS/src/SCAN-Planner/robot_api/zs_ros2/src/genisom_l1_control/config/twist.yaml" "智身 L1 配置"
elif [[ "$robot" == "go2" ]]; then
  check_file "$PROJECT_WS/src/SCAN-Planner/robot_api/unitree_ros2/setup_default.sh" "Unitree 环境"
  check_ros_package "$NAV_GO2_RMW_IMPLEMENTATION"
fi

if ((failures > 0)); then
  echo "环境检查失败，共 $failures 项异常。" >&2
  exit 1
fi

echo "环境检查通过。"
