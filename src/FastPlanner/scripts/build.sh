#!/usr/bin/env bash
set -euo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
project_root=$(cd -- "${script_dir}/.." && pwd)

set +u
source /opt/ros/humble/setup.bash
set -u
cd "${project_root}"
colcon build --symlink-install "$@"
