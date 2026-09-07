#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/lib/common.sh"
source_workspace

(($# >= 1)) || die "用法: $0 录包目录 [ros2 bag play 参数]"
bag_path="$(resolve_workspace_path "$1")"
shift
[[ -d "$bag_path" ]] || die "录包目录不存在: $bag_path"

banner "回放" "工程" "回放 ROS 2 Bag"
echo "录包目录: $bag_path"
exec_managed playback ros2 bag play "$bag_path" "$@"
