#!/usr/bin/env bash
set -eo pipefail

WS="${1:-$HOME/workspace/fastanchor-nav-stack}"
SRC_ROOT="$WS/src"

if [ ! -d "$SRC_ROOT" ]; then
  echo "[ERROR] Workspace src directory not found: $SRC_ROOT"
  exit 1
fi

NAV_LAUNCH="$(find "$SRC_ROOT" -type f -path '*/navigation_bringup/launch/navigation_system.launch.py' -print -quit)"

if [ -z "$NAV_LAUNCH" ]; then
  echo "[ERROR] navigation_system.launch.py not found under $SRC_ROOT"
  exit 1
fi

echo "[INFO] Target: $NAV_LAUNCH"

if grep -q 'run_windowed.launch.py' "$NAV_LAUNCH" && \
   grep -q 'scan_initial_path_topic' "$NAV_LAUNCH"; then
  echo "[INFO] Launch file already appears patched; skipping source modification."
else
  STAMP="$(date +%Y%m%d_%H%M%S)"
  cp "$NAV_LAUNCH" "${NAV_LAUNCH}.bak.${STAMP}"
  echo "[OK] Backup: ${NAV_LAUNCH}.bak.${STAMP}"

  python3 - "$NAV_LAUNCH" <<'PY'
from pathlib import Path
import sys

p = Path(sys.argv[1])
text = p.read_text()

old = '''    global_path_topic = LaunchConfiguration("global_path_topic")\n    local_target_distance = LaunchConfiguration("local_target_distance")\n'''
new = '''    global_path_topic = LaunchConfiguration("global_path_topic")\n    scan_initial_path_topic = LaunchConfiguration("scan_initial_path_topic")\n    path_window_size_x = LaunchConfiguration("path_window_size_x")\n    path_window_size_y = LaunchConfiguration("path_window_size_y")\n    path_cropper_update_rate = LaunchConfiguration("path_cropper_update_rate")\n    path_window_robot_aligned = LaunchConfiguration("path_window_robot_aligned")\n    local_target_distance = LaunchConfiguration("local_target_distance")\n'''
if old not in text:
    raise SystemExit('[ERROR] Cannot find LaunchConfiguration insertion point')
text = text.replace(old, new, 1)

old = '''        "run.launch.py",\n'''
new = '''        "run_windowed.launch.py",\n'''
if old not in text:
    raise SystemExit('[ERROR] Cannot find SCAN run.launch.py include')
text = text.replace(old, new, 1)

old = '''        DeclareLaunchArgument("global_path_topic", default_value="/planned_path"),\n        DeclareLaunchArgument(\n            "local_target_distance",\n'''
new = '''        DeclareLaunchArgument(\n            "global_path_topic",\n            default_value="/planned_path",\n            description="Stable full global path published by the global planner",\n        ),\n        DeclareLaunchArgument(\n            "scan_initial_path_topic",\n            default_value="/scan_planner/initial_path",\n            description="Window-cropped global path consumed by SCAN-Planner as initial_path",\n        ),\n        DeclareLaunchArgument(\n            "path_window_size_x",\n            default_value="5.0",\n            description="SCAN sliding-window X size used for global-path cropping (metres)",\n        ),\n        DeclareLaunchArgument(\n            "path_window_size_y",\n            default_value="5.0",\n            description="SCAN sliding-window Y size used for global-path cropping (metres)",\n        ),\n        DeclareLaunchArgument(\n            "path_cropper_update_rate",\n            default_value="20.0",\n            description="Global-path window cropper update rate in Hz",\n        ),\n        DeclareLaunchArgument(\n            "path_window_robot_aligned",\n            default_value="false",\n            description=(\n                "Whether the crop window rotates with robot yaw; must match the "\n                "SCAN-Planner sliding-window definition"\n            ),\n        ),\n        DeclareLaunchArgument(\n            "local_target_distance",\n'''
if old not in text:
    raise SystemExit('[ERROR] Cannot find global_path_topic declaration block')
text = text.replace(old, new, 1)

old = '''                "goal_topic": goal_topic,\n                "global_path_topic": global_path_topic,\n                "local_target_distance": local_target_distance,\n'''
new = '''                "goal_topic": goal_topic,\n                # Full FastPlanner path. run_windowed.launch.py crops this path before\n                # remapping the cropped segment into SCAN-Planner's `initial_path`.\n                "global_path_topic": global_path_topic,\n                "scan_initial_path_topic": scan_initial_path_topic,\n                "path_cropper_robot_frame": base_frame,\n                "path_window_size_x": path_window_size_x,\n                "path_window_size_y": path_window_size_y,\n                "path_cropper_update_rate": path_cropper_update_rate,\n                "path_window_robot_aligned": path_window_robot_aligned,\n                "local_target_distance": local_target_distance,\n'''
if old not in text:
    raise SystemExit('[ERROR] Cannot find SCAN launch_arguments insertion block')
text = text.replace(old, new, 1)

p.write_text(text)
print('[OK] Patched navigation_system.launch.py')
PY
fi

python3 -m py_compile "$NAV_LAUNCH"
echo "[OK] Python syntax check passed"

if [ ! -f "$WS/src/SCAN-Planner/planner/plan_manage/launch/run_windowed.launch.py" ]; then
  echo "[WARN] Expected run_windowed.launch.py not found at the common SCAN-Planner path."
  echo "       Verifying through package sources instead..."
  FOUND_WINDOWED="$(find "$SRC_ROOT" -type f -path '*/scan*/*' -name 'run_windowed.launch.py' -print -quit || true)"
  if [ -z "$FOUND_WINDOWED" ]; then
    echo "[ERROR] run_windowed.launch.py not found. Install the path cropper first."
    exit 1
  fi
  echo "[OK] Found: $FOUND_WINDOWED"
fi

set +u
source /opt/ros/humble/setup.bash
if [ -f "$WS/install/setup.bash" ]; then
  source "$WS/install/setup.bash"
fi
set -u

cd "$WS"
colcon build --symlink-install --packages-select scan_planner navigation_bringup

set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u

echo
ros2 pkg executables scan_planner | grep global_path_window_node >/dev/null || {
  echo "[ERROR] scan_planner/global_path_window_node is not installed"
  exit 1
}

echo "[OK] global_path_window_node executable visible"

echo
printf '%s\n' \
  "============================================================" \
  "READY" \
  "============================================================" \
  "Launch with:" \
  "  cd $WS" \
  "  source /opt/ros/humble/setup.bash" \
  "  source install/setup.bash" \
  "  ros2 launch navigation_bringup navigation_system.launch.py \\" \
  "    map_pcd_path:=\$PWD/maps/map_preprocessed2.pcd \\" \
  "    local_target_distance:=3.0 \\" \
  "    lidar_model:=mid360" \
  "" \
  "Default path flow:" \
  "  /planned_path -> global_path_window_node -> /scan_planner/initial_path -> SCAN-Planner"
