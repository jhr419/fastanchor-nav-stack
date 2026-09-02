#!/usr/bin/env bash
set -eo pipefail
set +u

# 从脚本所在位置进入工作区，避免依赖调用者当前目录。
cd "$(dirname "${BASH_SOURCE[0]}")/.."

if [[ ! -f "install/setup.bash" ]]; then
  echo "错误: 未找到 install/setup.bash，请先执行 ./scripts/build_all.sh" >&2
  exit 1
fi

source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

AUTO_CONFIRM=false
if [[ "${1:-}" == "--yes" ]]; then
  AUTO_CONFIRM=true
elif [[ $# -ne 0 ]]; then
  echo "用法: ./scripts/sdk_cmd.sh [--yes]" >&2
  exit 2
fi

echo "警告: 本脚本将请求 SDK 控制权，并让机器人站立、进入移动模式和开启速度桥。"
echo "请确认机器人位于平坦空旷场地，人员已远离，原厂遥控器和急停可用。"
if [[ "$AUTO_CONFIRM" != "true" ]]; then
  read -r -p "输入 1 表示已确认上述安全条件并继续，输入其他内容取消: " answer
  if [[ "$answer" != "1" ]]; then
    echo "已取消。"
    exit 0
  fi
fi

sdk_requested=false

call_service()
{
  local service_name="$1"
  local service_type="$2"
  local request="$3"
  local output

  if ! output=$(timeout 8 ros2 service call "$service_name" "$service_type" "$request" 2>&1); then
    echo "$output" >&2
    return 1
  fi
  echo "$output"
  if ! grep -q "success=True" <<< "$output"; then
    return 1
  fi
}

status_value()
{
  local wanted_key="$1"
  local output

  if ! output=$(timeout 5 ros2 topic echo --once /genisom/status 2>/dev/null); then
    return 1
  fi
  awk -v wanted_key="$wanted_key" '
    $0 ~ "^[[:space:]]*- key: " wanted_key "$" {
      getline
      sub(/^[[:space:]]*value:[[:space:]]*/, "")
      gsub(/\047/, "")
      print
      exit
    }
  ' <<< "$output"
}

wait_for_status()
{
  local key="$1"
  local expected="$2"
  local description="$3"
  local actual=""

  for _ in {1..20}; do
    actual=$(status_value "$key" || true)
    if [[ "$actual" == "$expected" ]]; then
      return 0
    fi
    sleep 0.2
  done

  echo "错误: 等待${description}超时，期望 ${key}=${expected}，实际 ${actual:-无法读取}" >&2
  return 1
}

safe_release()
{
  local exit_code="$1"
  trap - ERR INT TERM
  set +e
  if [[ "$sdk_requested" == "true" ]]; then
    echo "初始化未完成，正在关闭速度桥并释放到 REMOTE..." >&2
    timeout 5 ros2 service call /genisom/velocity_bridge/set_enabled \
      std_srvs/srv/SetBool '{data: false}' >/dev/null 2>&1
    timeout 5 ros2 service call /genisom/control/release_remote \
      std_srvs/srv/Trigger '{}' >/dev/null 2>&1
  fi
  exit "$exit_code"
}

trap 'safe_release $?' ERR
trap 'safe_release 130' INT
trap 'safe_release 143' TERM

if ! wait_for_status connected true "Manager 与机器人连接"; then
  echo "请确认 Manager 已启动，并检查 /genisom/status。" >&2
  exit 1
fi

if [[ "$(status_value estop_latched || true)" == "true" ]]; then
  echo
  echo "检测到 Manager ESTOP 已锁存。0 是软件急停，不是普通姿态，急停后不能直接请求 SDK。"
  echo "请先确认危险已经排除、原厂遥控器可用，并按原厂流程完成机器人恢复。"
  read -r -p "完成上述恢复后，输入 1 清除 Manager 锁存并继续，输入其他内容退出: " recovery_answer
  if [[ "$recovery_answer" != "1" ]]; then
    echo "未清除 ESTOP，脚本退出并保持 REMOTE。"
    exit 0
  fi
  call_service /genisom/clear_emergency_stop std_srvs/srv/Trigger '{}'
  wait_for_status control_owner REMOTE "REMOTE 控制权反馈"
  wait_for_status estop_latched false "Manager ESTOP 锁存解除"
  echo "Manager ESTOP 锁存已解除，准备重新请求 SDK。"
fi

call_service /genisom/control/request_sdk std_srvs/srv/Trigger '{}'
sdk_requested=true
wait_for_status control_owner SDK "SDK 控制权反馈"
wait_for_status pending_control_owner NONE "控制权切换完成"

call_service /genisom/mode/stand_up std_srvs/srv/Trigger '{}'

# 给站立动作留出完成时间，避免立即切换移动模式。
sleep 3
call_service /genisom/mode/move std_srvs/srv/Trigger '{}'
wait_for_status control_mode MOVE_MODE "移动模式反馈"

# 所有姿态命令都会关闭速度桥，因此必须最后开启。
call_service /genisom/velocity_bridge/set_enabled std_srvs/srv/SetBool '{data: true}'
wait_for_status velocity_bridge_enabled true "速度桥开启反馈"

echo
echo "SDK 速度控制已就绪。上层现在可以向 /cmd_vel 发布 geometry_msgs/msg/Twist。"

show_menu()
{
  echo
  echo "========== GENISOM 姿态与控制菜单 =========="
  echo "  0  锁存软件急停并释放到 REMOTE（不是普通姿态）"
  echo "  1  趴下/坐下"
  echo "  2  站立"
  echo "  3  进入移动模式并开启 cmd_vel 速度桥"
  echo "  4  平衡站立"
  echo "  5  锁定模式"
  echo "  i  显示统一状态"
  echo "  h  重新显示菜单"
  echo "  q  停车、关闭速度桥并切回 REMOTE"
  echo "============================================"
}

run_trigger_command()
{
  local service_name="$1"
  local label="$2"

  if call_service "$service_name" std_srvs/srv/Trigger '{}'; then
    echo "已执行: ${label}。姿态变化后速度桥保持关闭；需要行走时请输入 3。"
  else
    echo "执行失败: ${label}，请检查 /genisom/status。" >&2
  fi
}

release_to_remote_and_exit()
{
  set +e
  call_service /genisom/velocity_bridge/set_enabled std_srvs/srv/SetBool '{data: false}'
  if call_service /genisom/control/release_remote std_srvs/srv/Trigger '{}'; then
    wait_for_status control_owner REMOTE "REMOTE 控制权反馈"
  fi
  sdk_requested=false
  trap - ERR INT TERM
  echo "已关闭速度桥并释放到 REMOTE。"
  exit 0
}

show_menu
while true; do
  if ! read -r -p "请选择操作: " choice; then
    echo
    release_to_remote_and_exit
  fi

  case "$choice" in
    0)
      if call_service /genisom/emergency_stop std_srvs/srv/Trigger '{}'; then
        echo "已发送官方软件急停并锁存。正在释放到 REMOTE；下次运行会进入显式恢复流程。"
      fi
      release_to_remote_and_exit
      ;;
    1)
      run_trigger_command /genisom/mode/sit_down "趴下/坐下"
      ;;
    2)
      run_trigger_command /genisom/mode/stand_up "站立"
      ;;
    3)
      if call_service /genisom/mode/move std_srvs/srv/Trigger '{}' &&
        wait_for_status control_mode MOVE_MODE "移动模式反馈" &&
        call_service /genisom/velocity_bridge/set_enabled \
          std_srvs/srv/SetBool '{data: true}' &&
        wait_for_status velocity_bridge_enabled true "速度桥开启反馈"
      then
        echo "移动模式和 cmd_vel 速度桥已就绪，上层速度可以下发。"
      else
        echo "速度桥准备失败，请检查 /genisom/status。" >&2
      fi
      ;;
    4)
      run_trigger_command /genisom/mode/balance_stand "平衡站立"
      ;;
    5)
      run_trigger_command /genisom/mode/lock "锁定模式"
      ;;
    i|I)
      timeout 5 ros2 topic echo --once /genisom/status ||
        echo "读取 /genisom/status 失败。" >&2
      ;;
    h|H)
      show_menu
      ;;
    q|Q)
      release_to_remote_and_exit
      ;;
    *)
      echo "无效输入: ${choice}，输入 h 查看菜单。" >&2
      ;;
  esac
done
