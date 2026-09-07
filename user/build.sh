#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/lib/common.sh"
source_ros
require_command colcon

banner "构建" "工程" "Release 模式编译"
cd "$PROJECT_WS"

colcon build \
  --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  "$@"

echo "构建完成。请执行: source user/setup_env.sh"
