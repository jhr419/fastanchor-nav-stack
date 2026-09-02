#include "genisom_l1_control/manager_node.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>

#include "diagnostic_msgs/msg/key_value.hpp"

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

bool finite_twist(const geometry_msgs::msg::Twist & message)
{
  return std::isfinite(message.linear.x) && std::isfinite(message.linear.y) &&
         std::isfinite(message.angular.z);
}

std::uint8_t fault_level(const std::string & level)
{
  std::string normalized = level;
  std::transform(
    normalized.begin(), normalized.end(), normalized.begin(),
    [](unsigned char value) {return static_cast<char>(std::toupper(value));});
  if (normalized.find("ERROR") != std::string::npos ||
    normalized.find("FATAL") != std::string::npos)
  {
    return diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  }
  return diagnostic_msgs::msg::DiagnosticStatus::WARN;
}

const std::array<std::string, 16> & joint_names()
{
  static const std::array<std::string, 16> names{
    "left_front_abad_joint", "left_front_hip_joint", "left_front_knee_joint",
    "left_front_foot_joint", "right_front_abad_joint", "right_front_hip_joint",
    "right_front_knee_joint", "right_front_foot_joint", "left_rear_abad_joint",
    "left_rear_hip_joint", "left_rear_knee_joint", "left_rear_foot_joint",
    "right_rear_abad_joint", "right_rear_hip_joint", "right_rear_knee_joint",
    "right_rear_foot_joint"};
  return names;
}

}  // 匿名命名空间

ManagerNode::ManagerNode(const rclcpp::NodeOptions & options)
: Node("genisom_manager", options)
{
  const auto cmd_vel_topic = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
  const auto joint_states_topic =
    declare_parameter<std::string>("joint_states_topic", "/joint_states");
  const auto robot_ip = declare_parameter<std::string>("robot_ip", "192.168.168.168");
  const auto send_port = declare_parameter<int>("send_port", 8081);
  const auto recv_port = declare_parameter<int>("recv_port", 8080);
  cmd_vel_timeout_ms_ = declare_parameter<int>("cmd_vel_timeout_ms", 300);
  mode_switch_timeout_ms_ = declare_parameter<int>("mode_switch_timeout_ms", 2000);
  telemetry_rate_hz_ = declare_parameter<double>("telemetry_rate_hz", 20.0);
  status_rate_hz_ = declare_parameter<double>("status_rate_hz", 5.0);
  imu_frame_id_ = declare_parameter<std::string>("imu_frame_id", "imu_link");
  odom_frame_id_ = declare_parameter<std::string>("odom_frame_id", "odom");
  base_frame_id_ = declare_parameter<std::string>("base_frame_id", "base_link");

  publish_imu_enabled_ = declare_parameter<bool>("publish_imu", true);
  publish_odom_enabled_ = declare_parameter<bool>("publish_odom", true);
  publish_joint_states_enabled_ = declare_parameter<bool>("publish_joint_states", true);
  publish_battery_enabled_ = declare_parameter<bool>("publish_battery", true);
  publish_velocity_enabled_ = declare_parameter<bool>("publish_velocity", true);
  publish_motor_temperatures_enabled_ =
    declare_parameter<bool>("publish_motor_temperatures", true);
  publish_diagnostics_enabled_ = declare_parameter<bool>("publish_diagnostics", true);

  limits_.limit_cmd_vel_input = declare_parameter<bool>("limit_cmd_vel_input", false);
  limits_.max_linear_x = declare_parameter<double>("max_linear_x", 0.10);
  limits_.max_linear_y = declare_parameter<double>("max_linear_y", 0.05);
  limits_.max_angular_z = declare_parameter<double>("max_angular_z", 0.20);
  limits_.forward_joystick_per_mps =
    declare_parameter<double>("forward_joystick_per_mps", 1.0);
  limits_.lateral_joystick_per_mps =
    declare_parameter<double>("lateral_joystick_per_mps", 2.0);
  limits_.yaw_joystick_per_rps =
    declare_parameter<double>("yaw_joystick_per_rps", 0.5);
  limits_.max_forward_joystick = declare_parameter<double>("max_forward_joystick", 1.0);
  limits_.max_lateral_joystick = declare_parameter<double>("max_lateral_joystick", 1.0);
  limits_.max_yaw_joystick = declare_parameter<double>("max_yaw_joystick", 1.0);

  if (send_port <= 0 || send_port > 65535 || recv_port <= 0 || recv_port > 65535) {
    throw std::invalid_argument("SDK UDP 端口必须位于 1 到 65535");
  }
  if (cmd_vel_timeout_ms_ <= 0 || mode_switch_timeout_ms_ <= 0 ||
    telemetry_rate_hz_ <= 0.0 || status_rate_hz_ <= 0.0)
  {
    throw std::invalid_argument("超时和发布频率参数必须大于 0");
  }
  static_cast<void>(convert_twist_to_normalized(0.0, 0.0, 0.0, limits_));

  sdk_ = std::make_unique<SdkWrapper>(
    robot_ip, static_cast<std::uint32_t>(send_port), static_cast<std::uint32_t>(recv_port));

  create_publishers();
  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    cmd_vel_topic, rclcpp::QoS(10),
    std::bind(&ManagerNode::on_cmd_vel, this, std::placeholders::_1));
  if (publish_joint_states_enabled_) {
    joint_states_pub_ = create_publisher<sensor_msgs::msg::JointState>(
      joint_states_topic, rclcpp::SensorDataQoS());
  }
  create_services();
  create_timers();
  publish_connection(false);

  RCLCPP_INFO(
    get_logger(),
    "GENISOM Manager 已启动；默认不请求 SDK 控制权，速度桥默认关闭");
}

ManagerNode::~ManagerNode()
{
  std::lock_guard<std::mutex> lock(mutex_);
  try {
    if (!sdk_ || !sdk_->is_connected() || estop_latched_) {
      return;
    }
    const auto owner = sdk_->function_mode();
    if (owner == zsibot::FunctionMode::FM_SDK) {
      sdk_->stop_motion();
    }
    if (owner != zsibot::FunctionMode::FM_REMOTE) {
      sdk_->request_control(zsibot::FunctionMode::FM_REMOTE);
    }
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_logger(), "退出安全处理失败: %s", error.what());
  }
}

void ManagerNode::create_publishers()
{
  if (publish_imu_enabled_) {
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(
      "/genisom/imu", rclcpp::SensorDataQoS());
  }
  if (publish_odom_enabled_) {
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      "/genisom/odom", rclcpp::SensorDataQoS());
  }
  if (publish_battery_enabled_) {
    battery_pub_ = create_publisher<sensor_msgs::msg::BatteryState>(
      "/genisom/battery", rclcpp::QoS(5));
  }
  if (publish_velocity_enabled_) {
    velocity_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
      "/genisom/current_velocity", rclcpp::SensorDataQoS());
  }
  if (publish_motor_temperatures_enabled_) {
    motor_temperatures_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(
      "/genisom/motor_temperatures", rclcpp::SensorDataQoS());
  }

  const auto latched_qos = rclcpp::QoS(1).reliable().transient_local();
  model_pub_ = create_publisher<std_msgs::msg::String>("/genisom/model", latched_qos);
  version_pub_ = create_publisher<std_msgs::msg::String>("/genisom/version", latched_qos);
  connection_pub_ = create_publisher<std_msgs::msg::Bool>("/genisom/connection", latched_qos);
  status_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticStatus>(
    "/genisom/status", latched_qos);
  faults_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/genisom/faults", rclcpp::QoS(5));
  if (publish_diagnostics_enabled_) {
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", rclcpp::QoS(10));
  }
}

void ManagerNode::add_trigger_service(const std::string & name, TriggerCallback callback)
{
  trigger_services_.push_back(create_service<Trigger>(name, std::move(callback)));
}

void ManagerNode::add_command_service(
  const std::string & name, zsibot::CmdCode command, const std::string & label,
  CommandKind kind)
{
  add_trigger_service(
    name,
    [this, command, label, kind](const std::shared_ptr<Trigger::Request> request,
    std::shared_ptr<Trigger::Response> response)
    {
      on_sdk_command(command, label, kind, request, response);
    });
}

void ManagerNode::create_services()
{
  const auto add_owner_service =
    [this](const std::string & name, zsibot::FunctionMode target, bool clears_estop = false)
    {
      add_trigger_service(
        name,
        [this, target, clears_estop](const std::shared_ptr<Trigger::Request> request,
        std::shared_ptr<Trigger::Response> response)
        {
          on_request_control(target, clears_estop, request, response);
        });
    };

  add_owner_service("/genisom/control/request_sdk", zsibot::FunctionMode::FM_SDK);
  add_owner_service(
    "/genisom/control/request_general_sdk", zsibot::FunctionMode::FM_GENERAL_SDK);
  add_owner_service("/genisom/control/request_roamerx", zsibot::FunctionMode::FM_ROAMER);
  add_owner_service("/genisom/control/release_remote", zsibot::FunctionMode::FM_REMOTE);
  add_owner_service(
    "/genisom/clear_emergency_stop", zsibot::FunctionMode::FM_REMOTE, true);

  // 保留旧服务名，便于已有启动脚本迁移到 Manager。
  add_owner_service("/genisom/take_auto_control", zsibot::FunctionMode::FM_SDK);
  add_owner_service("/genisom/release_to_remote", zsibot::FunctionMode::FM_REMOTE);

  add_command_service(
    "/genisom/mode/stand_up", zsibot::CmdCode::CMD_STAND_UP, "站立", CommandKind::MODE);
  add_command_service(
    "/genisom/mode/sit_down", zsibot::CmdCode::CMD_SIT_DOWN, "趴下", CommandKind::MODE);
  add_command_service(
    "/genisom/mode/move", zsibot::CmdCode::CMD_MOVE_MODE, "移动模式", CommandKind::MODE);
  add_command_service(
    "/genisom/mode/balance_stand", zsibot::CmdCode::CMD_BALANCE_STAND_MODE,
    "平衡站立模式", CommandKind::MODE);
  add_command_service(
    "/genisom/mode/lock", zsibot::CmdCode::CMD_LOCK_MODE, "锁定模式", CommandKind::MODE);
  add_command_service(
    "/genisom/mode/enter_lab", zsibot::CmdCode::CMD_ENTER_LAB_MODE,
    "进入实验室模式", CommandKind::MODE);
  add_command_service(
    "/genisom/mode/exit_lab", zsibot::CmdCode::CMD_EXIT_LAB_MODE,
    "退出实验室模式", CommandKind::MODE);
  add_command_service(
    "/genisom/mode/enter_entertainment", zsibot::CmdCode::CMD_ENTER_ENTERTAINMENT_MODE,
    "进入文娱模式", CommandKind::MODE);
  add_command_service(
    "/genisom/mode/exit_entertainment", zsibot::CmdCode::CMD_EXIT_ENTERTAINMENT_MODE,
    "退出文娱模式", CommandKind::MODE);

  add_command_service(
    "/genisom/speed/slow", zsibot::CmdCode::CMD_SLOW_SPEED, "慢速档", CommandKind::SPEED);
  add_command_service(
    "/genisom/speed/normal", zsibot::CmdCode::CMD_NORMAL_SPEED,
    "正常速度档", CommandKind::SPEED);
  add_command_service(
    "/genisom/speed/fast", zsibot::CmdCode::CMD_FAST_SPEED, "快速档", CommandKind::SPEED);

  add_command_service(
    "/genisom/action/crawl_forward", zsibot::CmdCode::CMD_CRAWL_FORWARD,
    "匍匐前进", CommandKind::ACTION);
  add_command_service(
    "/genisom/action/climb_high_platform", zsibot::CmdCode::CMD_CLIMBING_HIGH_PLATFORM,
    "爬高台", CommandKind::ACTION);
  add_command_service(
    "/genisom/action/unload_squat", zsibot::CmdCode::CMD_UNLOAD_SQUAT,
    "卸货下蹲", CommandKind::ACTION);
  add_command_service(
    "/genisom/action/jump", zsibot::CmdCode::CMD_JUMP, "跳跃", CommandKind::ACTION);
  add_command_service(
    "/genisom/action/forward_jump", zsibot::CmdCode::CMD_FORWARD_JUMP,
    "前跳", CommandKind::ACTION);
  add_command_service(
    "/genisom/action/back_flip", zsibot::CmdCode::CMD_BACK_FLIP,
    "后空翻", CommandKind::ACTION);
  add_command_service(
    "/genisom/action/greet", zsibot::CmdCode::CMD_GREET, "招手", CommandKind::ACTION);
  add_command_service(
    "/genisom/action/two_leg_stand", zsibot::CmdCode::CMD_TWO_LEG_STAND,
    "双腿站立", CommandKind::ACTION);
  add_command_service(
    "/genisom/action/hand_stand", zsibot::CmdCode::CMD_HAND_STAND,
    "双腿倒立", CommandKind::ACTION);

  add_trigger_service(
    "/genisom/emergency_stop",
    std::bind(
      &ManagerNode::on_emergency_stop, this, std::placeholders::_1,
      std::placeholders::_2));
  velocity_bridge_service_ = create_service<SetBool>(
    "/genisom/velocity_bridge/set_enabled",
    std::bind(
      &ManagerNode::on_set_velocity_bridge, this, std::placeholders::_1,
      std::placeholders::_2));
}

void ManagerNode::create_timers()
{
  control_timer_ =
    create_wall_timer(kControlPeriod, std::bind(&ManagerNode::on_control_timer, this));
  const auto telemetry_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / telemetry_rate_hz_));
  const auto status_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / status_rate_hz_));
  telemetry_timer_ = create_wall_timer(
    telemetry_period, std::bind(&ManagerNode::on_telemetry_timer, this));
  status_timer_ = create_wall_timer(status_period, std::bind(&ManagerNode::on_status_timer, this));
}

void ManagerNode::on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr message)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!finite_twist(*message)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "丢弃包含 NaN 或 Inf 的 cmd_vel");
    return;
  }
  try {
    const auto owner = connected_ ? sdk_->function_mode() : zsibot::FunctionMode::FM_NULL;
    if (!velocity_bridge_may_send(
        connected_, velocity_bridge_enabled_, estop_latched_, pending_owner_.has_value(), owner))
    {
      if (velocity_bridge_enabled_ && owner != zsibot::FunctionMode::FM_SDK) {
        disable_velocity_bridge(false, "控制权不再属于 SDK");
      }
      return;
    }
    sdk_->send_normalized(
      convert_twist_to_normalized(
        message->linear.x, message->linear.y, message->angular.z, limits_));
    last_cmd_vel_time_ = std::chrono::steady_clock::now();
    have_fresh_cmd_vel_ = true;
    watchdog_stop_sent_ = false;
  } catch (const std::exception & error) {
    last_error_ = error.what();
  }
}

void ManagerNode::on_request_control(
  zsibot::FunctionMode target, bool clears_estop,
  const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> response)
{
  std::lock_guard<std::mutex> lock(mutex_);
  try {
    if (!connected_) {
      response->message = "SDK 未连接";
      return;
    }
    if (pending_owner_) {
      response->message = "已有控制权切换正在等待反馈";
      return;
    }
    if (estop_latched_ && target != zsibot::FunctionMode::FM_REMOTE) {
      response->message = "ESTOP 已锁存，只允许释放到 REMOTE";
      return;
    }

    const auto current = sdk_->function_mode();
    if (current == target) {
      if (clears_estop && target == zsibot::FunctionMode::FM_REMOTE) {
        estop_latched_ = false;
      }
      response->success = true;
      response->message = "控制权反馈已是 " + function_mode_name(target);
      return;
    }

    disable_velocity_bridge(current == zsibot::FunctionMode::FM_SDK, "开始切换控制权");
    sdk_->request_control(target);
    pending_owner_ = target;
    pending_owner_clears_estop_ = clears_estop;
    owner_transition_deadline_ = std::chrono::steady_clock::now() +
      std::chrono::milliseconds(mode_switch_timeout_ms_);
    last_command_ = "REQUEST_" + function_mode_name(target);
    response->success = true;
    response->message = "请求已发送，等待 FunctionMode 确认 " + function_mode_name(target);
  } catch (const std::exception & error) {
    last_error_ = error.what();
    response->message = last_error_;
  }
}

void ManagerNode::on_sdk_command(
  zsibot::CmdCode command, const std::string & label, CommandKind kind,
  const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> response)
{
  std::lock_guard<std::mutex> lock(mutex_);
  try {
    if (!connected_) {
      response->message = "SDK 未连接";
      return;
    }
    if (estop_latched_) {
      response->message = "ESTOP 已锁存，禁止发送运动状态或动作命令";
      return;
    }
    if (pending_owner_ || sdk_->function_mode() != zsibot::FunctionMode::FM_SDK) {
      response->message = "只有确认 FM_SDK 且无切换过程时才允许该命令";
      return;
    }
    if (kind == CommandKind::ACTION) {
      const auto current_model = sdk_->model();
      if (!command_supported_for_model(command, current_model)) {
        response->message = "官方型号矩阵禁止 " + model_name(current_model) + " 执行 " + label;
        return;
      }
      if (command_requires_lab_mode(command)) {
        const auto current_motion_mode = sdk_->motion_mode();
        if (current_motion_mode != zsibot::MotionMode::MM_REST &&
          current_motion_mode != zsibot::MotionMode::MM_RUNNING)
        {
          response->message = label + " 需要先确认进入官方实验室模式";
          return;
        }
      }
    }
    disable_velocity_bridge(true, label + " 命令执行前停车");
    sdk_->send_command(command);
    last_command_ = label;
    response->success = true;
    response->message = "已发送官方命令：" + label + "；请通过状态反馈确认结果";
  } catch (const std::exception & error) {
    last_error_ = error.what();
    response->message = last_error_;
  }
}

void ManagerNode::on_set_velocity_bridge(
  const std::shared_ptr<SetBool::Request> request,
  std::shared_ptr<SetBool::Response> response)
{
  std::lock_guard<std::mutex> lock(mutex_);
  try {
    if (!request->data) {
      const bool can_confirm_stop = connected_;
      disable_velocity_bridge(true, "用户关闭速度桥");
      response->success = true;
      response->message = can_confirm_stop ?
        "速度桥已关闭并停车" : "速度桥已关闭；SDK 断线，无法确认机器人已停车";
      return;
    }
    if (!connected_ || estop_latched_ || pending_owner_ ||
      sdk_->function_mode() != zsibot::FunctionMode::FM_SDK)
    {
      response->message = "只有已连接、无 ESTOP 且确认 FM_SDK 后才能开启速度桥";
      return;
    }
    sdk_->stop_motion();
    invalidate_cmd_vel();
    velocity_bridge_enabled_ = true;
    last_command_ = "ENABLE_VELOCITY_BRIDGE";
    response->success = true;
    response->message = "速度桥已开启，等待开启后的新 cmd_vel";
  } catch (const std::exception & error) {
    last_error_ = error.what();
    response->message = last_error_;
  }
}

void ManagerNode::on_emergency_stop(
  const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> response)
{
  std::lock_guard<std::mutex> lock(mutex_);
  try {
    if (!connected_) {
      response->message = "SDK 未连接，无法确认机器人收到急停";
      return;
    }
    disable_velocity_bridge(true, "急停");
    sdk_->send_command(zsibot::CmdCode::CMD_EMERGENCY_STOP);
    estop_latched_ = true;
    last_command_ = "EMERGENCY_STOP";
    response->success = true;
    response->message = "已发送官方 CMD_EMERGENCY_STOP 并锁存 Manager ESTOP";
  } catch (const std::exception & error) {
    last_error_ = error.what();
    response->message = last_error_;
  }
}

void ManagerNode::on_control_timer()
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto now = std::chrono::steady_clock::now();
  try {
    const bool connected = sdk_->is_connected();
    handle_connection(connected, now);
    if (!connected) {
      return;
    }
    const auto owner = sdk_->function_mode();
    handle_owner_feedback(owner, now);
    handle_cmd_vel_watchdog(now);
  } catch (const std::exception & error) {
    last_error_ = error.what();
  }
}

void ManagerNode::handle_connection(bool connected, SteadyTime now)
{
  if (connected == connected_) {
    return;
  }
  connected_ = connected;
  publish_connection(connected);
  if (!connected) {
    velocity_bridge_enabled_ = false;
    pending_owner_.reset();
    pending_owner_clears_estop_ = false;
    latest_snapshot_.reset();
    invalidate_cmd_vel();
    last_error_ = "SDK 连接丢失";
    RCLCPP_ERROR(get_logger(), "SDK 连接丢失，速度桥已关闭且不会自动重新抢权");
    return;
  }

  const auto owner = sdk_->function_mode();
  RCLCPP_INFO(
    get_logger(), "SDK 已连接，FunctionMode=%s", function_mode_name(owner).c_str());
  if (owner == zsibot::FunctionMode::FM_SDK) {
    sdk_->stop_motion();
    sdk_->request_control(zsibot::FunctionMode::FM_REMOTE);
    pending_owner_ = zsibot::FunctionMode::FM_REMOTE;
    owner_transition_deadline_ = now + std::chrono::milliseconds(mode_switch_timeout_ms_);
    last_command_ = "RECONNECT_RELEASE_REMOTE";
  }
  last_error_.clear();
}

void ManagerNode::handle_owner_feedback(zsibot::FunctionMode owner, SteadyTime now)
{
  if (last_owner_ == zsibot::FunctionMode::FM_SDK &&
    owner != zsibot::FunctionMode::FM_SDK)
  {
    disable_velocity_bridge(false, "SDK 控制权已被其他控制者接管");
  }

  if (pending_owner_ && owner == *pending_owner_) {
    const auto confirmed = *pending_owner_;
    pending_owner_.reset();
    if (confirmed == zsibot::FunctionMode::FM_REMOTE && pending_owner_clears_estop_) {
      estop_latched_ = false;
    }
    pending_owner_clears_estop_ = false;
    last_error_.clear();
    RCLCPP_INFO(
      get_logger(), "控制权反馈已确认: %s", function_mode_name(confirmed).c_str());
  } else if (pending_owner_ && now >= owner_transition_deadline_) {
    const auto timed_out_target = *pending_owner_;
    pending_owner_.reset();
    pending_owner_clears_estop_ = false;
    last_error_ = "控制权切换超时，目标=" + function_mode_name(timed_out_target) +
      "，当前=" + function_mode_name(owner);
    if (timed_out_target != zsibot::FunctionMode::FM_REMOTE) {
      if (owner == zsibot::FunctionMode::FM_SDK) {
        sdk_->stop_motion();
      }
      sdk_->request_control(zsibot::FunctionMode::FM_REMOTE);
      pending_owner_ = zsibot::FunctionMode::FM_REMOTE;
      owner_transition_deadline_ = now + std::chrono::milliseconds(mode_switch_timeout_ms_);
    }
  }
  last_owner_ = owner;
}

void ManagerNode::handle_cmd_vel_watchdog(SteadyTime now)
{
  if (!velocity_bridge_enabled_ || !have_fresh_cmd_vel_ || watchdog_stop_sent_ ||
    last_owner_ != zsibot::FunctionMode::FM_SDK)
  {
    return;
  }
  const auto age = std::chrono::duration_cast<std::chrono::milliseconds>(
    now - last_cmd_vel_time_).count();
  if (age <= cmd_vel_timeout_ms_) {
    return;
  }
  sdk_->stop_motion();
  have_fresh_cmd_vel_ = false;
  watchdog_stop_sent_ = true;
  RCLCPP_WARN(get_logger(), "cmd_vel 超过 %d ms 未更新，已停车", cmd_vel_timeout_ms_);
}

void ManagerNode::disable_velocity_bridge(bool send_stop, const std::string & reason)
{
  if (send_stop && connected_ && sdk_->function_mode() == zsibot::FunctionMode::FM_SDK) {
    sdk_->stop_motion();
  }
  if (velocity_bridge_enabled_) {
    RCLCPP_INFO(get_logger(), "速度桥已关闭: %s", reason.c_str());
  }
  velocity_bridge_enabled_ = false;
  invalidate_cmd_vel();
}

void ManagerNode::invalidate_cmd_vel()
{
  have_fresh_cmd_vel_ = false;
  watchdog_stop_sent_ = false;
}

void ManagerNode::on_telemetry_timer()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!connected_) {
    return;
  }
  try {
    latest_snapshot_ = sdk_->read_snapshot();
    publish_telemetry(*latest_snapshot_, get_clock()->now());
  } catch (const std::exception & error) {
    last_error_ = error.what();
  }
}

void ManagerNode::publish_telemetry(
  const RobotSnapshot & snapshot, const rclcpp::Time & stamp)
{
  if (imu_pub_) {publish_imu(snapshot, stamp);}
  if (odom_pub_) {publish_odom(snapshot, stamp);}
  if (joint_states_pub_) {publish_joint_states(snapshot, stamp);}
  if (battery_pub_) {publish_battery(snapshot, stamp);}
  if (velocity_pub_) {publish_velocity(snapshot, stamp);}
  if (motor_temperatures_pub_) {publish_motor_temperatures(snapshot);}
  publish_device_info(snapshot);
}

void ManagerNode::publish_imu(const RobotSnapshot & snapshot, const rclcpp::Time & stamp)
{
  sensor_msgs::msg::Imu message;
  message.header.stamp = stamp;
  message.header.frame_id = imu_frame_id_;
  message.orientation.w = snapshot.quaternion[0];
  message.orientation.x = snapshot.quaternion[1];
  message.orientation.y = snapshot.quaternion[2];
  message.orientation.z = snapshot.quaternion[3];
  message.angular_velocity.x = snapshot.body_gyro[0];
  message.angular_velocity.y = snapshot.body_gyro[1];
  message.angular_velocity.z = snapshot.body_gyro[2];
  message.linear_acceleration.x = snapshot.body_acceleration[0];
  message.linear_acceleration.y = snapshot.body_acceleration[1];
  message.linear_acceleration.z = snapshot.body_acceleration[2];
  imu_pub_->publish(message);
}

void ManagerNode::publish_odom(const RobotSnapshot & snapshot, const rclcpp::Time & stamp)
{
  nav_msgs::msg::Odometry message;
  message.header.stamp = stamp;
  message.header.frame_id = odom_frame_id_;
  message.child_frame_id = base_frame_id_;
  message.pose.pose.position.x = snapshot.position[0];
  message.pose.pose.position.y = snapshot.position[1];
  message.pose.pose.position.z = snapshot.position[2];
  message.pose.pose.orientation.w = snapshot.quaternion[0];
  message.pose.pose.orientation.x = snapshot.quaternion[1];
  message.pose.pose.orientation.y = snapshot.quaternion[2];
  message.pose.pose.orientation.z = snapshot.quaternion[3];
  message.twist.twist.linear.x = snapshot.body_velocity[0];
  message.twist.twist.linear.y = snapshot.body_velocity[1];
  message.twist.twist.linear.z = snapshot.body_velocity[2];
  message.twist.twist.angular.z = snapshot.speed.angle_speed;
  odom_pub_->publish(message);
}

void ManagerNode::publish_joint_states(
  const RobotSnapshot & snapshot, const rclcpp::Time & stamp)
{
  sensor_msgs::msg::JointState message;
  message.header.stamp = stamp;
  message.name.assign(joint_names().begin(), joint_names().end());
  message.position.resize(16);
  message.velocity.resize(16);
  message.effort.resize(16);
  for (std::size_t leg = 0; leg < 4; ++leg) {
    for (std::size_t joint = 0; joint < 4; ++joint) {
      const std::size_t output_index = leg * 4 + joint;
      message.position[output_index] = snapshot.joint_position[joint][leg];
      message.velocity[output_index] = snapshot.joint_velocity[joint][leg];
      message.effort[output_index] = snapshot.joint_effort[joint][leg];
    }
  }
  joint_states_pub_->publish(message);
}

void ManagerNode::publish_battery(
  const RobotSnapshot & snapshot, const rclcpp::Time & stamp)
{
  sensor_msgs::msg::BatteryState message;
  message.header.stamp = stamp;
  message.voltage = snapshot.battery.volt;
  message.current = snapshot.battery.current;
  message.temperature = snapshot.battery.temp;
  message.percentage = snapshot.battery.power >= 0 && snapshot.battery.power <= 100 ?
    static_cast<float>(snapshot.battery.power) / 100.0F :
    std::numeric_limits<float>::quiet_NaN();
  message.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
  message.power_supply_health = snapshot.battery.error == 0 ?
    sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_GOOD :
    sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
  message.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
  message.present = snapshot.connected;
  battery_pub_->publish(message);
}

void ManagerNode::publish_velocity(
  const RobotSnapshot & snapshot, const rclcpp::Time & stamp)
{
  geometry_msgs::msg::TwistStamped message;
  message.header.stamp = stamp;
  message.header.frame_id = base_frame_id_;
  message.twist.linear.x = snapshot.speed.speed;
  message.twist.linear.y = snapshot.speed.shift_speed;
  message.twist.angular.z = snapshot.speed.angle_speed;
  velocity_pub_->publish(message);
}

void ManagerNode::publish_motor_temperatures(const RobotSnapshot & snapshot)
{
  std_msgs::msg::Float32MultiArray message;
  message.layout.dim.resize(1);
  message.layout.dim[0].label = "raw_motor_index";
  message.layout.dim[0].size = snapshot.motor_temperatures.size();
  message.layout.dim[0].stride = snapshot.motor_temperatures.size();
  message.data.assign(snapshot.motor_temperatures.begin(), snapshot.motor_temperatures.end());
  motor_temperatures_pub_->publish(message);
}

void ManagerNode::publish_device_info(const RobotSnapshot & snapshot)
{
  std_msgs::msg::String model_message;
  model_message.data = model_name(snapshot.model);
  model_pub_->publish(model_message);
  std_msgs::msg::String version_message;
  version_message.data = "mc=" + snapshot.version.mc_version +
    "; dog_task=" + snapshot.version.dog_task_version;
  version_pub_->publish(version_message);
}

diagnostic_msgs::msg::DiagnosticStatus ManagerNode::build_manager_status(SteadyTime now) const
{
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "GENISOM L1-W ROS2 Manager";
  status.hardware_id = latest_snapshot_ ? latest_snapshot_->serial_number : "unknown";

  std::int64_t cmd_vel_age_ms = -1;
  if (have_fresh_cmd_vel_) {
    cmd_vel_age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      now - last_cmd_vel_time_).count();
  }
  const auto owner = connected_ ? last_owner_ : zsibot::FunctionMode::FM_NULL;
  const auto control_mode = latest_snapshot_ ? latest_snapshot_->control_mode :
    zsibot::ControlMode::CM_NULL;
  const auto speed_level = latest_snapshot_ ? latest_snapshot_->speed_level :
    zsibot::SpeedLevel::SL_NULL;
  const auto motion_mode = latest_snapshot_ ? latest_snapshot_->motion_mode :
    zsibot::MotionMode::MM_NULL;
  const auto motion_type = latest_snapshot_ ? latest_snapshot_->motion_type :
    zsibot::MotionType::MT_NULL;
  const std::size_t fault_count = latest_snapshot_ ? latest_snapshot_->faults.size() : 0;

  status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  status.message = "ready";
  if (!connected_) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = "disconnected";
  } else if (estop_latched_) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = "estop_latched";
  } else if (fault_count > 0 || !last_error_.empty()) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = fault_count > 0 ? "robot_fault" : "manager_warning";
  }

  status.values = {
    key_value("connected", bool_name(connected_)),
    key_value("control_owner", function_mode_name(owner)),
    key_value("control_mode", control_mode_name(control_mode)),
    key_value("speed_level", speed_level_name(speed_level)),
    key_value("motion_mode", motion_mode_name(motion_mode)),
    key_value("motion_type", motion_type_name(motion_type)),
    key_value("velocity_bridge_enabled", bool_name(velocity_bridge_enabled_)),
    key_value("cmd_vel_input_limit_enabled", bool_name(limits_.limit_cmd_vel_input)),
    key_value(
      "forward_joystick_per_mps", std::to_string(limits_.forward_joystick_per_mps)),
    key_value(
      "lateral_joystick_per_mps", std::to_string(limits_.lateral_joystick_per_mps)),
    key_value("yaw_joystick_per_rps", std::to_string(limits_.yaw_joystick_per_rps)),
    key_value("estop_latched", bool_name(estop_latched_)),
    key_value("cmd_vel_age_ms", std::to_string(cmd_vel_age_ms)),
    key_value("fault_count", std::to_string(fault_count)),
    key_value(
      "fault_summary",
      fault_count > 0 ? latest_snapshot_->faults.front().info : "none"),
    key_value(
      "pending_control_owner",
      pending_owner_ ? function_mode_name(*pending_owner_) : "NONE"),
    key_value("model", latest_snapshot_ ? model_name(latest_snapshot_->model) : "UNKNOWN"),
    key_value(
      "serial_number", latest_snapshot_ ? latest_snapshot_->serial_number : ""),
    key_value("device_name", latest_snapshot_ ? latest_snapshot_->device_name : ""),
    key_value("wifi_ssid", latest_snapshot_ ? latest_snapshot_->wifi_ssid : ""),
    key_value(
      "device_temperature_c",
      latest_snapshot_ ? std::to_string(latest_snapshot_->temperature) : "nan"),
    key_value(
      "battery_power_percent",
      latest_snapshot_ ? std::to_string(latest_snapshot_->battery.power) : "-1"),
    key_value(
      "forward_speed_mps",
      latest_snapshot_ ? std::to_string(latest_snapshot_->speed.speed) : "nan"),
    key_value(
      "lateral_speed_mps",
      latest_snapshot_ ? std::to_string(latest_snapshot_->speed.shift_speed) : "nan"),
    key_value(
      "yaw_rate_rps",
      latest_snapshot_ ? std::to_string(latest_snapshot_->speed.angle_speed) : "nan"),
    key_value("mc_version", latest_snapshot_ ? latest_snapshot_->version.mc_version : ""),
    key_value(
      "dog_task_version",
      latest_snapshot_ ? latest_snapshot_->version.dog_task_version : ""),
    key_value("last_command", last_command_.empty() ? "none" : last_command_),
    key_value("last_error", last_error_.empty() ? "none" : last_error_)};
  return status;
}

diagnostic_msgs::msg::DiagnosticArray ManagerNode::build_fault_diagnostics(
  const rclcpp::Time & stamp) const
{
  diagnostic_msgs::msg::DiagnosticArray message;
  message.header.stamp = stamp;
  if (!latest_snapshot_ || latest_snapshot_->faults.empty()) {
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.name = "GENISOM faults";
    status.hardware_id = latest_snapshot_ ? latest_snapshot_->serial_number : "unknown";
    status.message = "no faults";
    message.status.push_back(status);
    return message;
  }
  for (const auto & fault : latest_snapshot_->faults) {
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.level = fault_level(fault.level);
    status.name = "GENISOM fault/" + fault.module + "/" + fault.submodule;
    status.hardware_id = latest_snapshot_->serial_number;
    status.message = fault.info;
    status.values = {
      key_value("error_code", std::to_string(fault.error_code)),
      key_value("reported_level", fault.level)};
    message.status.push_back(status);
  }
  return message;
}

void ManagerNode::on_status_timer()
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto manager_status = build_manager_status(std::chrono::steady_clock::now());
  status_pub_->publish(manager_status);
  const auto fault_diagnostics = build_fault_diagnostics(get_clock()->now());
  faults_pub_->publish(fault_diagnostics);
  if (diagnostics_pub_) {
    auto diagnostics = fault_diagnostics;
    diagnostics.status.insert(diagnostics.status.begin(), manager_status);
    diagnostics_pub_->publish(diagnostics);
  }
}

void ManagerNode::publish_connection(bool connected)
{
  std_msgs::msg::Bool message;
  message.data = connected;
  connection_pub_->publish(message);
}

}  // 命名空间 genisom_l1_control
