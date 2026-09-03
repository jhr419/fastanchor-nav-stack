#include "genisom_l1_control/twist_node.hpp"

#include <cmath>
#include <cstdint>
#include <stdexcept>

#include "diagnostic_msgs/msg/key_value.hpp"
#include "genisom_l1_control/process_supervisor.hpp"

namespace genisom_l1_control
{
namespace
{

constexpr auto kControlPeriod = std::chrono::milliseconds(20);
constexpr std::uint32_t kHighLevelStandMode = 1U;
constexpr std::uint32_t kHighLevelMoveMode = 3U;

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
  robot_ip_ = declare_parameter<std::string>("robot_ip", "192.168.168.168");
  local_ip_ = declare_parameter<std::string>("local_ip", "192.168.168.99");
  local_port_ = declare_parameter<int>("local_port", 43988);
  const auto cmd_vel_topic = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");

  connection_timeout_ms_ = declare_parameter<int>("connection_timeout_ms", 10000);
  mode_command_timeout_ms_ = declare_parameter<int>("mode_command_timeout_ms", 8000);
  command_retry_ms_ = declare_parameter<int>("command_retry_ms", 500);
  status_rate_hz_ = declare_parameter<double>("status_rate_hz", 5.0);
  auto_stand_ = declare_parameter<bool>("auto_stand", true);

  limits_.limit_cmd_vel_input = declare_parameter<bool>("limit_cmd_vel_input", false);
  limits_.max_linear_x = declare_parameter<double>("max_linear_x", 3.7);
  limits_.max_linear_y = declare_parameter<double>("max_linear_y", 1.0);
  limits_.max_angular_z = declare_parameter<double>("max_angular_z", 3.0);
  limits_.linear_deadband = declare_parameter<double>("linear_deadband", 0.0);
  limits_.angular_deadband = declare_parameter<double>("angular_deadband", 0.0);

  if (local_port_ <= 0 || local_port_ > 65535) {
    throw std::invalid_argument("HighLevel 本地 UDP 端口必须位于 1 到 65535");
  }
  if (connection_timeout_ms_ <= 0 || mode_command_timeout_ms_ <= 0 ||
    command_retry_ms_ <= 0 || status_rate_hz_ <= 0.0)
  {
    throw std::invalid_argument("超时、重试周期和状态发布频率必须大于 0");
  }
  static_cast<void>(convert_twist_to_velocity(0.0, 0.0, 0.0, limits_));

  velocity_client_ =
    std::make_unique<HighLevelVelocityClient>(local_ip_, local_port_, robot_ip_);

  const auto latched_qos = rclcpp::QoS(1).reliable().transient_local();
  status_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticStatus>(
    "/genisom/twist/status", latched_qos);
  ready_pub_ = create_publisher<std_msgs::msg::Bool>("/genisom/twist/ready", latched_qos);
  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    cmd_vel_topic, rclcpp::QoS(10),
    std::bind(&TwistNode::on_cmd_vel, this, std::placeholders::_1));

  control_timer_ =
    create_wall_timer(kControlPeriod, std::bind(&TwistNode::on_control_timer, this));
  const auto status_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / status_rate_hz_));
  status_timer_ = create_wall_timer(status_period, std::bind(&TwistNode::on_status_timer, this));

  node_started_at_ = std::chrono::steady_clock::now();
  state_deadline_ = node_started_at_ + std::chrono::milliseconds(connection_timeout_ms_);
  publish_ready(false);
  publish_status();

  RCLCPP_INFO(
    get_logger(),
    "Twist 控制节点已启动：使用旧版 HighLevel::move(vx, vy, yaw_rate) 直接下发速度");
  RCLCPP_WARN(
    get_logger(),
    "请确认机器人端 sdk_config.yaml 的 target_ip/target_port 与 local_ip/local_port 一致；"
    "旧版 HighLevel 接口不提供遥控器接管反馈");
}

TwistNode::~TwistNode()
{
  try {
    if (!velocity_client_ || !connected_) {
      return;
    }
    velocity_client_->stop_motion();
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
    connected_ = velocity_client_->is_connected();
    if (!connected_) {
      if (connected_once_) {
        request_shutdown("HighLevel SDK 连接丢失", false);
      } else if (now >= state_deadline_) {
        request_shutdown("等待 HighLevel SDK 连接超时", false);
      }
      return;
    }

    last_control_mode_ = velocity_client_->control_mode();

    if (!connected_once_) {
      connected_once_ = true;
      RCLCPP_INFO(get_logger(), "HighLevel SDK 已连接");
      handle_wait_connection(now);
    }

    switch (state_) {
      case State::WAIT_CONNECTION:
        handle_wait_connection(now);
        break;
      case State::WAIT_STAND:
        handle_wait_stand(now, last_control_mode_);
        break;
      case State::ACTIVE:
        break;
      case State::SHUTTING_DOWN:
        break;
    }
  } catch (const std::exception & error) {
    request_shutdown(std::string("HighLevel SDK 调用失败: ") + error.what(), true);
  }
}

void TwistNode::handle_wait_connection(SteadyTime now)
{
  if (state_ != State::WAIT_CONNECTION) {
    return;
  }
  last_error_.clear();
  if (auto_stand_) {
    begin_stand_sequence(now);
    return;
  }
  activate_velocity_control();
}

void TwistNode::begin_stand_sequence(SteadyTime now)
{
  have_cmd_vel_ = false;
  last_velocity_command_ = VelocityCommand{};
  velocity_client_->stand_up();
  state_ = State::WAIT_STAND;
  state_deadline_ = now + std::chrono::milliseconds(mode_command_timeout_ms_);
  last_command_sent_at_ = now;
  RCLCPP_INFO(get_logger(), "已发送 HighLevel::standUp，等待控制状态进入 STAND 或 MOVE");
}

void TwistNode::handle_wait_stand(SteadyTime now, std::uint32_t control_mode)
{
  if (control_mode == kHighLevelStandMode || control_mode == kHighLevelMoveMode) {
    activate_velocity_control();
    return;
  }
  if (now >= state_deadline_) {
    request_shutdown(
      "等待站立状态超时，当前 HighLevel 控制状态=" +
      highlevel_control_mode_name(control_mode), true);
    return;
  }
  if (now - last_command_sent_at_ >= std::chrono::milliseconds(command_retry_ms_)) {
    velocity_client_->stand_up();
    last_command_sent_at_ = now;
  }
}

void TwistNode::activate_velocity_control()
{
  state_ = State::ACTIVE;
  ready_ = true;
  have_cmd_vel_ = false;
  last_velocity_command_ = VelocityCommand{};
  last_error_.clear();
  publish_ready(true);
  RCLCPP_INFO(
    get_logger(),
    "Twist 控制已就绪：收到一条 /cmd_vel 立即调用一次 HighLevel::move");
}

void TwistNode::on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr message)
{
  if (!std::isfinite(message->linear.x) || !std::isfinite(message->linear.y) ||
    !std::isfinite(message->angular.z))
  {
    RCLCPP_ERROR(get_logger(), "拒绝包含非有限 linear.x、linear.y 或 angular.z 的 cmd_vel");
    return;
  }
  if (!ready_ || state_ != State::ACTIVE) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "cmd_vel 已忽略：HighLevel SDK 尚未连接或未完成站立确认");
    return;
  }

  try {
    const auto command = convert_twist_to_velocity(
      message->linear.x, message->linear.y, message->angular.z, limits_);
    velocity_client_->send_velocity(command);
    last_velocity_command_ = command;
    last_cmd_vel_at_ = std::chrono::steady_clock::now();
    have_cmd_vel_ = true;
  } catch (const std::exception & error) {
    request_shutdown(std::string("cmd_vel 下发失败: ") + error.what(), true);
  }
}

void TwistNode::request_shutdown(const std::string & reason, bool send_stop)
{
  if (shutdown_requested_) {
    return;
  }
  shutdown_requested_ = true;
  state_ = State::SHUTTING_DOWN;
  ready_ = false;
  last_error_ = reason;
  publish_ready(false);

  if (send_stop && connected_) {
    try {
      velocity_client_->stop_motion();
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "退出前 HighLevel 停车失败: %s", error.what());
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
    key_value("sdk_backend", "legacy_highlevel_move"),
    key_value("connected", bool_name(connected_)),
    key_value("robot_ip", robot_ip_),
    key_value("local_ip", local_ip_),
    key_value("local_port", std::to_string(local_port_)),
    key_value("control_mode", highlevel_control_mode_name(last_control_mode_)),
    key_value("ready", bool_name(ready_)),
    key_value("cmd_vel_age_ms", std::to_string(cmd_vel_age_ms)),
    key_value("cmd_vel_send_policy", "on_message"),
    key_value("cmd_vel_timeout_ms", "disabled"),
    key_value("linear_x_enabled", "true"),
    key_value("linear_y_enabled", "true"),
    key_value("angular_z_enabled", "true"),
    key_value("exclusive_translation_rotation", "false"),
    key_value("cmd_vel_input_limit_enabled", bool_name(limits_.limit_cmd_vel_input)),
    key_value("max_linear_x", std::to_string(limits_.max_linear_x)),
    key_value("max_linear_y", std::to_string(limits_.max_linear_y)),
    key_value("max_angular_z", std::to_string(limits_.max_angular_z)),
    key_value("linear_deadband", std::to_string(limits_.linear_deadband)),
    key_value("angular_deadband", std::to_string(limits_.angular_deadband)),
    key_value("last_linear_x", std::to_string(last_velocity_command_.linear_x)),
    key_value("last_linear_y", std::to_string(last_velocity_command_.linear_y)),
    key_value("last_angular_z", std::to_string(last_velocity_command_.angular_z)),
    key_value("remote_takeover_feedback", "unavailable"),
    key_value("last_error", last_error_.empty() ? "none" : last_error_)};
  status_pub_->publish(message);
}

std::string TwistNode::state_name(State state)
{
  switch (state) {
    case State::WAIT_CONNECTION: return "WAIT_CONNECTION";
    case State::WAIT_STAND: return "WAIT_STAND";
    case State::ACTIVE: return "ACTIVE";
    case State::SHUTTING_DOWN: return "SHUTTING_DOWN";
    default: return "UNKNOWN";
  }
}

}  // 命名空间 genisom_l1_control
