#!/usr/bin/env bash
set -euo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
project_root=$(cd -- "${script_dir}/.." && pwd)

set +u
source /opt/ros/humble/setup.bash
source "${project_root}/install/setup.bash"
set -u
export FAST_PLANNER_ROOT="${project_root}"
cd "${project_root}"
exec ros2 run fast_global_planner fast_planner_feasibility_test "$@"
