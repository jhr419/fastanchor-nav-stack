#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/lib/common.sh"
source_workspace
require_command colcon

banner "测试" "工程" "执行回归测试"
cd "$PROJECT_WS"

echo "检查工程管理脚本..."
bash -n user/*.sh user/lib/*.sh setup.bash
user/start_all.sh --dry-run --robot none >/dev/null
echo "工程管理脚本检查通过。"

if [[ "${1:-}" == "--all" ]]; then
  shift
  colcon test --event-handlers console_direct+ "$@"
else
  if (($# > 0)); then
    packages=("$@")
  else
    packages=(
      fast_anchor_fusion
      fast_anchor_localization
      genisom_l1_control
      nav3d_global_planning
      navigation_bringup
      scan_planner
    )
  fi
  colcon test --packages-select "${packages[@]}" --event-handlers console_direct+
fi

colcon test-result --verbose
