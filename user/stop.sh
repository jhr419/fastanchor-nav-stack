#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/lib/common.sh"

banner "停止" "工程" "停止脚本管理的进程"
mkdir -p "$RUN_DIR"
shopt -s nullglob
pid_files=("$RUN_DIR"/*.pid)

if ((${#pid_files[@]} == 0)); then
  echo "没有已登记的运行进程。"
  exit 0
fi

live_pid_files=()
for pid_file in "${pid_files[@]}"; do
  read -r pid recorded_start module < "$pid_file" || true
  current_start="$(process_start_time "${pid:-}" 2>/dev/null || true)"
  if [[ -z "${pid:-}" || -z "${recorded_start:-}" || "$current_start" != "$recorded_start" ]]; then
    echo "清理失效记录: $(basename "$pid_file")"
    rm -f "$pid_file"
    continue
  fi

  echo "发送 SIGINT: $module (PID=$pid)"
  kill -INT "$pid" 2>/dev/null || true
  live_pid_files+=("$pid_file")
done

deadline=$((SECONDS + 10))
while ((SECONDS < deadline)); do
  remaining=0
  for pid_file in "${live_pid_files[@]}"; do
    [[ -f "$pid_file" ]] || continue
    read -r pid recorded_start module < "$pid_file" || true
    if [[ "$(process_start_time "${pid:-}" 2>/dev/null || true)" == "${recorded_start:-missing}" ]]; then
      remaining=$((remaining + 1))
    fi
  done
  ((remaining == 0)) && break
  sleep 1
done

for pid_file in "${live_pid_files[@]}"; do
  [[ -f "$pid_file" ]] || continue
  read -r pid recorded_start module < "$pid_file" || true
  if [[ "$(process_start_time "${pid:-}" 2>/dev/null || true)" == "${recorded_start:-missing}" ]]; then
    echo "进程未在 10 秒内退出，发送 SIGTERM: $module (PID=$pid)"
    kill -TERM "$pid" 2>/dev/null || true
  else
    echo "已停止: $module"
  fi
  rm -f "$pid_file"
done

echo "停止流程完成。"
