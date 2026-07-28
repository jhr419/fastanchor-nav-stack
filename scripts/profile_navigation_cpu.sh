#!/usr/bin/env bash
set -euo pipefail

duration_s="${1:-30}"
if ! [[ "${duration_s}" =~ ^[1-9][0-9]*$ ]]; then
  echo "Usage: $0 [positive-duration-seconds]" >&2
  exit 2
fi

pattern='fast_anchor_localization_node|scan_planner_node|astar_global_planner_node|closed_loop_controller|laserMapping|livox_ros_driver2|rviz2'
mapfile -t navigation_pids < <(pgrep -f "${pattern}" || true)
if (( ${#navigation_pids[@]} == 0 )); then
  echo "No navigation processes found. Start navigation_system.launch.py first." >&2
  exit 1
fi

echo "Sampling navigation CPU threads for ${duration_s}s."
echo "Columns: elapsed_s pid tid last_cpu cpu_percent command"
echo "Inherited CPU affinity:"
for pid in "${navigation_pids[@]}"; do
  allowed_cpus="$(awk '/^Cpus_allowed_list:/ {print $2}' "/proc/${pid}/status")"
  command_name="$(ps -p "${pid}" -o comm=)"
  echo "  pid=${pid} command=${command_name} allowed_cpus=${allowed_cpus}"
done
echo

for ((second = 1; second <= duration_s; ++second)); do
  for pid in "${navigation_pids[@]}"; do
    if [[ -r "/proc/${pid}/stat" ]]; then
      ps -L -p "${pid}" -o pid=,tid=,psr=,pcpu=,comm= | \
        awk -v elapsed="${second}" '{print elapsed, $0}'
    fi
  done
  if (( second < duration_s )); then
    sleep 1
  fi
done

echo
echo "Per-process snapshot:"
ps -p "$(IFS=,; echo "${navigation_pids[*]}")" -o pid=,psr=,pcpu=,pmem=,rss=,comm=,args=
