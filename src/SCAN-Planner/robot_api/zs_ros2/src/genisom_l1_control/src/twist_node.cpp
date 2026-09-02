#include "genisom_l1_control/twist_node.hpp"

#include <array>
#include <cmath>
#include <stdexcept>
#include <utility>

#include "diagnostic_msgs/msg/key_value.hpp"
#include "genisom_l1_control/process_supervisor.hpp"

namespace genisom_l1_control
{
namespace
{

constexpr auto kControlPeriod = std::chrono::milliseconds(20);

diagnostic_msgs::msg::KeyValue key_value(const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue result;
  result.key = key;
  result.value = value;
  return result;
}

std::string bool_name(bool value)
{
  return value ? "true" : "false";
}

}  // 匿名命名空间

TwistNode::TwistNode(const rclcpp::NodeOptions & options)
: Node("genisom_twist_control", options)
{
  const auto robot_ip = declare_parameter<std::string>("robot_ip", "192.168.168.168");
  const auto send_port = declare_parameter<int>("send_port", 8081);
  const auto recv_port = declare_parameter<int>("recv_port", 8080);
  const auto cmd_vel_topic = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");

  cmd_vel_timeout_ms_ = declare_parameter<int>("cmd_vel_timeout_ms", 300);
  connection_timeout_ms_ = declare_parameter<int>("connection_timeout_ms", 10000);
  disconnect_grace_ms_ = declare_parameter<int>("disconnect_grace_ms", 10000);
  control_request_timeout_ms_ =
    declare_parameter<int>("control_request_timeout_ms", 5000);
  mode_command_timeout_ms_ = declare_parameter<int>("mode_command_timeout_ms", 8000);
  command_retry_ms_ = declare_parameter<int>("command_retry_ms", 500);
  status_rate_hz_ = declare_parameter<double>("status_rate_hz", 5.0);
  auto_stand_ = declare_parameter<bool>("auto_stand", true);
  auto_move_mode_ = declare_parameter<bool>("auto_move_mode", true);
  exit_on_control_loss_ = declare_parameter<bool>("exit_on_control_loss", true);

  limits_.limit_cmd_vel_input = declare_parameter<bool>("limit_cmd_vel_input", false);
  limits_.exclusive_translation_rotation =
    declare_parameter<bool>("exclusive_translation_rotation", true);
  limits_.max_linear_x = declare_parameter<double>("max_linear_x", 0.30);
  limits_.max_angular_z = declare_parameter<double>("max_angular_z", 0.50);
  limits_.forward_joystick_per_mps =
    declare_parameter<double>("forward_joystick_per_mps", 1.0);
  limits_.yaw_joystick_per_rps =
    declare_parameter<double>("yaw_joystick_per_rps", 0.5);
  limits_.max_forward_joystick = declare_parameter<double>("max_forward_joystick", 1.0);
  limits_.max_yaw_joystick = declare_parameter<double>("max_yaw_joystick", 1.0);
  limits_.linear_deadband = declare_parameter<double>("linear_deadband", 0.01);
  limits_.angular_deadband = declare_parameter<double>("angular_deadband", 0.05);

  if (send_port <= 0 || send_port > 65535 || recv_port <= 0 || recv_port > 65535) {
    throw std::invalid_argument("SDK UDP 端口必须位于 1 到 65535");
  }
  if (cmd_vel_timeout_ms_ <= 0 || connection_timeout_ms_ <= 0 || disconnect_grace_ms_ <= 0 ||
    control_request_timeout_ms_ <= 0 || mode_command_timeout_ms_ <= 0 ||
    command_retry_ms_ <= 0 || status_rate_hz_ <= 0.0)
  {
    throw std::invalid_argument("超时、重试周期和状态发布频率必须大于 0");
  }
  static_cast<void>(convert_twist_to_longitudinal(0.0, 0.0, limits_));

  sdk_ = std::make_unique<SdkWrapper>(
    robot_ip, static_cast<std::uint32_t>(send_port), static_cast<std::uint32_t>(recv_port));

  const auto latched_qos = rclcpp::QoS(1).reliable().transient_local();
  status_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticStatus>(
    "/genisom/twist/status", latched_qos);
  ready_pub_ = create_publisher<std_msgs::msg::Bool>("/genisom/twist/ready", latched_qos);
  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    cmd_vel_topic, rclcpp::QoS(10),
    std::bind(&TwistNode::on_cmd_vel, this, std::placeholders::_1));

  control_timer_ = create_wall_timer(kControlPeriod, std::bind(&TwistNode::on_control_timer, this));
  const auto status_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / status_rate_hz_));
  status_timer_ = create_wall_timer(status_period, std::bind(&TwistNode::on_status_timer, this));

  node_started_at_ = std::chrono::steady_clock::now();
  state_deadline_ = node_started_at_ + std::chrono::milliseconds(connection_timeout_ms_);
  publish_ready(false);
  publish_status();

  RCLCPP_INFO(
    get_logger(),
    "Twist 控制节点已启动：将自动请求 SDK、站立并进入移动模式；"
    "仅启用 linear.x 与 angular.z，平移与转向互斥");
  RCLCPP_WARN(
    get_logger(),
    "此节点必须独占官方 SDK；遥控器接管后将退出且不会自动重新抢权");
}

TwistNode::~TwistNode()
{
  try {
    if (!sdk_ || !connected_ || last_owner_ != zsibot::FunctionMode::FM_SDK) {
      return;
    }
    sdk_->stop_motion();
    sdk_->request_control(zsibot::FunctionMode::FM_REMOTE);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_logger(), "Twist 节点退出安全处理失败: %s", error.what());
  }
}

void TwistNode::on_control_timer()
{
  if (shutdown_requested_) {
    return;
  }

  const auto now = std::chrono::steady_clock::now();
  try {
    connected_ = sdk_->is_connected();
    if (!connected_) {
      if (connected_once_) {
        if (!disconnected_since_) {
          disconnected_since_ = now;
          ready_ = false;
          have_cmd_vel_ = false;
          watchdog_stop_sent_ = true;
          active_command_ = NormalizedCommand{};
          last_error_ = "SDK 心跳暂时中断";
          publish_ready(false);
          RCLCPP_WARN(
            get_logger(), "SDK 心跳暂时中断，进入 %d ms 宽限期并持续发送零速",
            disconnect_grace_ms_);
        }

        sdk_->stop_motion();
        const auto disconnected_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
          now - *disconnected_since_).count();
        if (disconnected_ms >= disconnect_grace_ms_) {
          request_shutdown("SDK 连接持续丢失且超过宽限期；不会自动重新抢权", false);
        }
      } else if (now >= state_deadline_) {
        request_shutdown("等待 SDK 连接超时", false);
      }
      return;
    }

    const bool recovered_from_disconnect = disconnected_since_.has_value();
    disconnected_since_.reset();
    const auto owner = sdk_->function_mode();
    const auto control_mode = sdk_->control_mode();
    last_owner_ = owner;
    last_control_mode_ = control_mode;

    if (!connected_once_) {
      connected_once_ = true;
      RCLCPP_INFO(get_logger(), "SDK 已连接，开始请求 SDK 控制权");
      enter_wait_sdk_control(now);
    }

    if (sdk_control_lost(sdk_control_confirmed_, owner)) {
      ready_ = false;
      publish_ready(false);
      if (exit_on_control_loss_) {
        request_shutdown(
          "控制权已由 " + function_mode_name(owner) +
          " 接管；停止 SDK 发送并释放连接，不自动重新抢权",
          false);
      }
      return;
    }

    if (recovered_from_disconnect) {
      last_error_.clear();
      if (state_ == State::ACTIVE) {
        ready_ = true;
        publish_ready(true);
      }
      RCLCPP_INFO(get_logger(), "SDK 心跳已恢复，保持当前控制权且等待新的 cmd_vel");
    }

    switch (state_) {
      case State::WAIT_CONNECTION:
        enter_wait_sdk_control(now);
        break;
      case State::WAIT_SDK_CONTROL:
        handle_wait_sdk_control(now, owner);
        break;
      case State::WAIT_STAND:
        handle_wait_stand(now, control_mode);
        break;
      case State::WAIT_MOVE_MODE:
        handle_wait_move_mode(now, control_mode);
        break;
      case State::ACTIVE:
        handle_cmd_vel_watchdog(now);
        sdk_->send_normalized(active_command_);
        break;
      case State::SHUTTING_DOWN:
        break;
    }
  } catch (const std::exception & error) {
    request_shutdown(std::string("SDK 调用失败: ") + error.what(), true);
  }
}

void TwistNode::enter_wait_sdk_control(SteadyTime now)
{
  state_ = State::WAIT_SDK_CONTROL;
  state_deadline_ = now + std::chrono::milliseconds(control_request_timeout_ms_);
  last_command_sent_at_ = now;
  sdk_->request_control(zsibot::FunctionMode::FM_SDK);
  RCLCPP_INFO(get_logger(), "已发送 CMD_SDK_CONTROL_RIGHT，等待 FunctionMode=SDK");
}

void TwistNode::handle_wait_sdk_control(SteadyTime now, zsibot::FunctionMode owner)
{
  if (owner == zsibot::FunctionMode::FM_SDK) {
    sdk_control_confirmed_ = true;
    RCLCPP_INFO(get_logger(), "SDK 控制权已确认");
    begin_motion_mode_sequence(now, last_control_mode_);
    return;
  }

  if (now >= state_deadline_) {
    request_shutdown(
      "请求 SDK 控制权超时，当前 FunctionMode=" + function_mode_name(owner), false);
    return;
  }

  if (now - last_command_sent_at_ >= std::chrono::milliseconds(command_retry_ms_)) {
    sdk_->request_control(zsibot::FunctionMode::FM_SDK);
    last_command_sent_at_ = now;
  }
}

void TwistNode::begin_motion_mode_sequence(SteadyTime now, zsibot::ControlMode control_mode)
{
  have_cmd_vel_ = false;
  watchdog_stop_sent_ = false;
  active_command_ = NormalizedCommand{};

  if (control_mode == zsibot::ControlMode::CM_MOVE_MODE) {
    activate_velocity_control();
    return;
  }
  if (auto_stand_) {
    sdk_->send_command(zsibot::CmdCode::CMD_STAND_UP);
    state_ = State::WAIT_STAND;
    state_deadline_ = now + std::chrono::milliseconds(mode_command_timeout_ms_);
    last_command_sent_at_ = now;
    RCLCPP_INFO(get_logger(), "已发送站立命令，等待 ControlMode=STAND_UP");
    return;
  }
  if (auto_move_mode_) {
    sdk_->send_command(zsibot::CmdCode::CMD_MOVE_MODE);
    state_ = State::WAIT_MOVE_MODE;
    state_deadline_ = now + std::chrono::milliseconds(mode_command_timeout_ms_);
    last_command_sent_at_ = now;
    RCLCPP_INFO(get_logger(), "已发送移动模式命令，等待 ControlMode=MOVE_MODE");
    return;
  }
  activate_velocity_control();
}

void TwistNode::handle_wait_stand(SteadyTime now, zsibot::ControlMode control_mode)
{
  if (control_mode == zsibot::ControlMode::CM_MOVE_MODE) {
    activate_velocity_control();
    return;
  }
  if (control_mode == zsibot::ControlMode::CM_STAND_UP) {
    if (auto_move_mode_) {
      sdk_->send_command(zsibot::CmdCode::CMD_MOVE_MODE);
      state_ = State::WAIT_MOVE_MODE;
      state_deadline_ = now + std::chrono::milliseconds(mode_command_timeout_ms_);
      last_command_sent_at_ = now;
      RCLCPP_INFO(get_logger(), "站立已确认，已发送移动模式命令");
    } else {
      activate_velocity_control();
    }
    return;
  }
  if (now >= state_deadline_) {
    request_shutdown(
      "等待站立状态超时，当前 ControlMode=" + control_mode_name(control_mode), true);
    return;
  }
  if (now - last_command_sent_at_ >= std::chrono::milliseconds(command_retry_ms_)) {
    sdk_->send_command(zsibot::CmdCode::CMD_STAND_UP);
    last_command_sent_at_ = now;
  }
}

void TwistNode::handle_wait_move_mode(SteadyTime now, zsibot::ControlMode control_mode)
{
  if (control_mode == zsibot::ControlMode::CM_MOVE_MODE) {
    activate_velocity_control();
    return;
  }
  if (now >= state_deadline_) {
    request_shutdown(
      "等待移动模式超时，当前 ControlMode=" + control_mode_name(control_mode), true);
    return;
  }
  if (now - last_command_sent_at_ >= std::chrono::milliseconds(command_retry_ms_)) {
    sdk_->send_command(zsibot::CmdCode::CMD_MOVE_MODE);
    last_command_sent_at_ = now;
  }
}

void TwistNode::activate_velocity_control()
{
  state_ = State::ACTIVE;
  ready_ = true;
  have_cmd_vel_ = false;
  watchdog_stop_sent_ = false;
  last_error_.clear();
  publish_ready(true);
  RCLCPP_INFO(
    get_logger(),
    "Twist 控制已就绪：接收 /cmd_vel，仅执行 linear.x 与 angular.z，linear.y 永久禁用");
}

void TwistNode::on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr message)
{
  if (!std::isfinite(message->linear.x) || !std::isfinite(message->angular.z)) {
    RCLCPP_ERROR(get_logger(), "拒绝包含非有限 linear.x 或 angular.z 的 cmd_vel");
    return;
  }
  if (std::abs(message->linear.y) > 1.0e-6) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "收到非零 linear.y=%f，但 Twist 节点已在 SDK 层强制禁用横向移动",
      message->linear.y);
  }
  if (!ready_ || state_ != State::ACTIVE || last_owner_ != zsibot::FunctionMode::FM_SDK) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "cmd_vel 已忽略：尚未完成 SDK 控制权和移动模式确认");
    return;
  }

  try {
    const auto command = convert_twist_to_longitudinal(
      message->linear.x, message->angular.z, limits_);
    if (limits_.exclusive_translation_rotation &&
      std::abs(message->linear.x) > limits_.linear_deadband &&
      std::abs(message->angular.z) > limits_.angular_deadband)
    {
      RCLCPP_DEBUG_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "收到平移和转向组合命令，已按 yaw 优先规则抑制 linear.x");
    }
    active_command_ = command;
    last_cmd_vel_at_ = std::chrono::steady_clock::now();
    have_cmd_vel_ = true;
    watchdog_stop_sent_ = false;
  } catch (const std::exception & error) {
    request_shutdown(std::string("cmd_vel 下发失败: ") + error.what(), true);
  }
}

void TwistNode::handle_cmd_vel_watchdog(SteadyTime now)
{
  if (!have_cmd_vel_ || watchdog_stop_sent_) {
    return;
  }
  const auto age = std::chrono::duration_cast<std::chrono::milliseconds>(
    now - last_cmd_vel_at_).count();
  if (age <= cmd_vel_timeout_ms_) {
    return;
  }
  active_command_ = NormalizedCommand{};
  have_cmd_vel_ = false;
  watchdog_stop_sent_ = true;
  RCLCPP_WARN(get_logger(), "cmd_vel 超过 %d ms 未更新，已发送零速停车", cmd_vel_timeout_ms_);
}

void TwistNode::request_shutdown(const std::string & reason, bool release_remote)
{
  if (shutdown_requested_) {
    return;
  }
  shutdown_requested_ = true;
  state_ = State::SHUTTING_DOWN;
  ready_ = false;
  last_error_ = reason;
  publish_ready(false);

  if (release_remote && connected_ && last_owner_ == zsibot::FunctionMode::FM_SDK) {
    try {
      sdk_->stop_motion();
      sdk_->request_control(zsibot::FunctionMode::FM_REMOTE);
      last_owner_ = zsibot::FunctionMode::FM_REMOTE;
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "退出前停车或释放 REMOTE 失败: %s", error.what());
    }
  }

  publish_status();
  RCLCPP_WARN(get_logger(), "%s；Twist 节点正在退出", reason.c_str());
  request_supervised_shutdown();
  rclcpp::shutdown();
}

void TwistNode::on_status_timer()
{
  publish_status();
}

void TwistNode::publish_ready(bool ready)
{
  if (!ready_pub_) {
    return;
  }
  std_msgs::msg::Bool message;
  message.data = ready;
  ready_pub_->publish(message);
}

void TwistNode::publish_status()
{
  if (!status_pub_) {
    return;
  }
  diagnostic_msgs::msg::DiagnosticStatus message;
  message.name = "GENISOM Twist SDK Control";
  message.hardware_id = "GENISOM-L1-W";
  message.level = ready_ ? diagnostic_msgs::msg::DiagnosticStatus::OK :
    diagnostic_msgs::msg::DiagnosticStatus::WARN;
  message.message = ready_ ? "ready" : state_name(state_);

  std::int64_t cmd_vel_age_ms = -1;
  if (have_cmd_vel_) {
    cmd_vel_age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - last_cmd_vel_at_).count();
  }
  message.values = {
    key_value("state", state_name(state_)),
    key_value("connected", bool_name(connected_)),
    key_value("control_owner", function_mode_name(last_owner_)),
    key_value("control_mode", control_mode_name(last_control_mode_)),
    key_value("ready", bool_name(ready_)),
    key_value("cmd_vel_age_ms", std::to_string(cmd_vel_age_ms)),
    key_value("linear_x_enabled", "true"),
    key_value("linear_y_enabled", "false"),
    key_value("angular_z_enabled", "true"),
    key_value(
      "exclusive_translation_rotation",
      bool_name(limits_.exclusive_translation_rotation)),
    key_value("linear_deadband", std::to_string(limits_.linear_deadband)),
    key_value("angular_deadband", std::to_string(limits_.angular_deadband)),
    key_value("disconnect_grace_ms", std::to_string(disconnect_grace_ms_)),
    key_value("auto_reacquire_sdk", "false"),
    key_value("exit_on_control_loss", bool_name(exit_on_control_loss_)),
    key_value("last_error", last_error_.empty() ? "none" : last_error_)};
  status_pub_->publish(message);
}

std::string TwistNode::state_name(State state)
{
  switch (state) {
    case State::WAIT_CONNECTION: return "WAIT_CONNECTION";
    case State::WAIT_SDK_CONTROL: return "WAIT_SDK_CONTROL";
    case State::WAIT_STAND: return "WAIT_STAND";
    case State::WAIT_MOVE_MODE: return "WAIT_MOVE_MODE";
    case State::ACTIVE: return "ACTIVE";
    case State::SHUTTING_DOWN: return "SHUTTING_DOWN";
    default: return "UNKNOWN";
  }
}

}  // 命名空间 genisom_l1_control
