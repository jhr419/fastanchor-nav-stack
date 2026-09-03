#include "genisom_l1_control/sdk_wrapper.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace genisom_l1_control
{
namespace
{

float scale_axis(
  double value, bool limit_input, double input_limit, double joystick_gain,
  double joystick_limit)
{
  if (!std::isfinite(value)) {
    return 0.0F;
  }
  const double input = limit_input ? std::clamp(value, -input_limit, input_limit) : value;
  return static_cast<float>(
    std::clamp(input * joystick_gain, -joystick_limit, joystick_limit));
}

bool is_wheel_model(zsibot::Model model)
{
  return model == zsibot::Model::MODEL_XGW || model == zsibot::Model::MODEL_XGWHSPD;
}

}  // 匿名命名空间

NormalizedCommand convert_twist_to_normalized(
  double linear_x, double linear_y, double angular_z, const CommandLimits & limits)
{
  if (limits.limit_cmd_vel_input &&
    (!std::isfinite(limits.max_linear_x) || !std::isfinite(limits.max_linear_y) ||
    !std::isfinite(limits.max_angular_z) || limits.max_linear_x <= 0.0 ||
    limits.max_linear_y <= 0.0 || limits.max_angular_z <= 0.0))
  {
    throw std::invalid_argument("启用 cmd_vel 输入限幅时，物理速度上限必须大于 0");
  }
  if (!std::isfinite(limits.forward_joystick_per_mps) ||
    !std::isfinite(limits.lateral_joystick_per_mps) ||
    !std::isfinite(limits.yaw_joystick_per_rps) ||
    limits.forward_joystick_per_mps <= 0.0 || limits.lateral_joystick_per_mps <= 0.0 ||
    limits.yaw_joystick_per_rps <= 0.0)
  {
    throw std::invalid_argument("摇杆标定增益必须是有限正数");
  }
  if (!std::isfinite(limits.max_forward_joystick) ||
    !std::isfinite(limits.max_lateral_joystick) ||
    !std::isfinite(limits.max_yaw_joystick) || limits.max_forward_joystick <= 0.0 ||
    limits.max_forward_joystick > 1.0 || limits.max_lateral_joystick <= 0.0 ||
    limits.max_lateral_joystick > 1.0 || limits.max_yaw_joystick <= 0.0 ||
    limits.max_yaw_joystick > 1.0)
  {
    throw std::invalid_argument("摇杆上限必须位于 (0, 1]");
  }

  return NormalizedCommand{
    scale_axis(
      linear_x, limits.limit_cmd_vel_input, limits.max_linear_x,
      limits.forward_joystick_per_mps, limits.max_forward_joystick),
    scale_axis(
      angular_z, limits.limit_cmd_vel_input, limits.max_angular_z,
      limits.yaw_joystick_per_rps, limits.max_yaw_joystick),
    scale_axis(
      linear_y, limits.limit_cmd_vel_input, limits.max_linear_y,
      limits.lateral_joystick_per_mps, limits.max_lateral_joystick)};
}

bool command_supported_for_model(zsibot::CmdCode command, zsibot::Model model)
{
  if (model == zsibot::Model::MODEL_XG) {
    return command != zsibot::CmdCode::CMD_CRAWL_FORWARD &&
           command != zsibot::CmdCode::CMD_CLIMBING_HIGH_PLATFORM &&
           command != zsibot::CmdCode::CMD_HAND_STAND;
  }
  if (is_wheel_model(model)) {
    return command != zsibot::CmdCode::CMD_JUMP &&
           command != zsibot::CmdCode::CMD_FORWARD_JUMP &&
           command != zsibot::CmdCode::CMD_BACK_FLIP &&
           command != zsibot::CmdCode::CMD_GREET &&
           command != zsibot::CmdCode::CMD_TWO_LEG_STAND;
  }
  return false;
}

bool command_requires_lab_mode(zsibot::CmdCode command)
{
  return command == zsibot::CmdCode::CMD_JUMP ||
         command == zsibot::CmdCode::CMD_FORWARD_JUMP ||
         command == zsibot::CmdCode::CMD_BACK_FLIP ||
         command == zsibot::CmdCode::CMD_GREET ||
         command == zsibot::CmdCode::CMD_TWO_LEG_STAND ||
         command == zsibot::CmdCode::CMD_UNLOAD_SQUAT;
}

bool velocity_bridge_may_send(
  bool connected, bool bridge_enabled, bool estop_latched, bool transition_pending,
  zsibot::FunctionMode owner)
{
  return connected && bridge_enabled && !estop_latched && !transition_pending &&
         owner == zsibot::FunctionMode::FM_SDK;
}

bool sdk_control_lost(bool sdk_control_confirmed, zsibot::FunctionMode owner)
{
  return sdk_control_confirmed && owner != zsibot::FunctionMode::FM_SDK;
}

SdkWrapper::SdkWrapper(
  const std::string & robot_ip, std::uint32_t send_port, std::uint32_t recv_port)
: executor_(std::make_unique<zsibot::ZsibotExecutor>(
      zsibot::Role::ROLE_SDK, robot_ip, send_port, recv_port))
{
}

SdkWrapper::~SdkWrapper() = default;

bool SdkWrapper::is_connected() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return executor_->IsConnected();
}

zsibot::FunctionMode SdkWrapper::function_mode() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return executor_->GetFunctionMode();
}

zsibot::ControlMode SdkWrapper::control_mode() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return executor_->GetControlMode();
}

zsibot::MotionMode SdkWrapper::motion_mode() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return executor_->GetMotionMode();
}

zsibot::Model SdkWrapper::model() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return executor_->GetModel();
}

RobotSnapshot SdkWrapper::read_snapshot() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  RobotSnapshot snapshot;
  snapshot.connected = executor_->IsConnected();
  snapshot.power = executor_->GetPower();
  snapshot.temperature = executor_->GetTemperature();
  snapshot.serial_number = executor_->GetSn();
  snapshot.device_name = executor_->GetDevName();
  snapshot.wifi_ssid = executor_->GetWifiSsid();
  snapshot.version = executor_->GetVersion();
  snapshot.speed = executor_->GetSpeed();
  snapshot.battery = executor_->GetBatteryInfo();
  snapshot.model = executor_->GetModel();
  snapshot.speed_level = executor_->GetSpeedLevel();
  snapshot.function_mode = executor_->GetFunctionMode();
  snapshot.control_mode = executor_->GetControlMode();
  snapshot.motion_mode = executor_->GetMotionMode();
  snapshot.motion_type = executor_->GetMotionType();
  snapshot.quaternion = executor_->GetQuaternion();
  snapshot.rpy = executor_->GetRPY();
  snapshot.body_acceleration = executor_->GetBodyAcc();
  snapshot.body_gyro = executor_->GetBodyGyro();
  snapshot.position = executor_->GetPosition();
  snapshot.world_velocity = executor_->GetWorldVelocity();
  snapshot.body_velocity = executor_->GetBodyVelocity();
  snapshot.motor_temperatures = executor_->GetMotorTemp();
  snapshot.joint_position = {
    executor_->GetLegAbadJoint(), executor_->GetLegHipJoint(),
    executor_->GetLegKneeJoint(), executor_->GetLegFootJoint()};
  snapshot.joint_velocity = {
    executor_->GetLegAbadJointVel(), executor_->GetLegHipJointVel(),
    executor_->GetLegKneeJointVel(), executor_->GetLegFootJointVel()};
  snapshot.joint_effort = {
    executor_->GetLegAbadJointTorque(), executor_->GetLegHipJointTorque(),
    executor_->GetLegKneeJointTorque(), executor_->GetLegFootJointTorque()};
  snapshot.faults = executor_->GetFaultInfo();
  return snapshot;
}

void SdkWrapper::request_control(zsibot::FunctionMode target)
{
  zsibot::CmdCode command = zsibot::CmdCode::CMD_NULL;
  switch (target) {
    case zsibot::FunctionMode::FM_REMOTE:
      command = zsibot::CmdCode::CMD_REMOTE_CONTROL_RIGHT;
      break;
    case zsibot::FunctionMode::FM_SDK:
      command = zsibot::CmdCode::CMD_SDK_CONTROL_RIGHT;
      break;
    case zsibot::FunctionMode::FM_GENERAL_SDK:
      command = zsibot::CmdCode::CMD_GENERAL_SDK_CONTROL_RIGHT;
      break;
    case zsibot::FunctionMode::FM_ROAMER:
      command = zsibot::CmdCode::CMD_ROAMERX_CONTROL_RIGHT;
      break;
    default:
      throw std::invalid_argument("不支持请求该 FunctionMode");
  }
  send_command(command);
}

void SdkWrapper::send_command(zsibot::CmdCode command)
{
  std::lock_guard<std::mutex> lock(mutex_);
  executor_->SetCmd(command);
}

void SdkWrapper::send_normalized(const NormalizedCommand & command)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const std::array<zsibot::float32_t, 4> joystick{
    command.forward, command.yaw, command.lateral, 0.0F};
  executor_->SetRemote(joystick, std::array<zsibot::float32_t, 14>{});
}

void SdkWrapper::stop_motion()
{
  send_normalized(NormalizedCommand{});
}

std::string function_mode_name(zsibot::FunctionMode mode)
{
  switch (mode) {
    case zsibot::FunctionMode::FM_REMOTE: return "REMOTE";
    case zsibot::FunctionMode::FM_TRACE: return "TRACE";
    case zsibot::FunctionMode::FM_SDK: return "SDK";
    case zsibot::FunctionMode::FM_GENERAL_SDK: return "GENERAL_SDK";
    case zsibot::FunctionMode::FM_ROAMER: return "ROAMERX";
    case zsibot::FunctionMode::FM_NULL:
    default: return "NULL";
  }
}

std::string control_mode_name(zsibot::ControlMode mode)
{
  switch (mode) {
    case zsibot::ControlMode::CM_STAND_UP: return "STAND_UP";
    case zsibot::ControlMode::CM_SIT_DOWN: return "SIT_DOWN";
    case zsibot::ControlMode::CM_LOCK_MODE: return "LOCK_MODE";
    case zsibot::ControlMode::CM_EMERGENCY_STOP: return "EMERGENCY_STOP";
    case zsibot::ControlMode::CM_MOVE_MODE: return "MOVE_MODE";
    case zsibot::ControlMode::CM_BALANCE_STAND_MODE: return "BALANCE_STAND_MODE";
    case zsibot::ControlMode::CM_INIT: return "INIT";
    case zsibot::ControlMode::CM_NULL:
    default: return "NULL";
  }
}

std::string speed_level_name(zsibot::SpeedLevel level)
{
  switch (level) {
    case zsibot::SpeedLevel::SL_SLOW: return "SLOW";
    case zsibot::SpeedLevel::SL_NORMAL: return "NORMAL";
    case zsibot::SpeedLevel::SL_FAST: return "FAST";
    case zsibot::SpeedLevel::SL_NULL:
    default: return "NULL";
  }
}

std::string motion_mode_name(zsibot::MotionMode mode)
{
  switch (mode) {
    case zsibot::MotionMode::MM_RUNNING: return "RUNNING";
    case zsibot::MotionMode::MM_REST: return "REST";
    case zsibot::MotionMode::MM_FORBID: return "FORBID";
    case zsibot::MotionMode::MM_NULL:
    default: return "NULL";
  }
}

std::string motion_type_name(zsibot::MotionType type)
{
  switch (type) {
    case zsibot::MotionType::MT_JUMP: return "JUMP";
    case zsibot::MotionType::MT_FORWARD_JUMP: return "FORWARD_JUMP";
    case zsibot::MotionType::MT_BACK_FLIP: return "BACK_FLIP";
    case zsibot::MotionType::MT_GREET: return "GREET";
    case zsibot::MotionType::MT_TWO_LEG_STAND: return "TWO_LEG_STAND";
    case zsibot::MotionType::MT_FORBID: return "FORBID";
    case zsibot::MotionType::MT_UNKNOWN: return "UNKNOWN";
    case zsibot::MotionType::MT_UNLOADING_SQUAT: return "UNLOAD_SQUAT";
    case zsibot::MotionType::MT_NULL:
    default: return "NULL";
  }
}

std::string model_name(zsibot::Model model)
{
  switch (model) {
    case zsibot::Model::MODEL_XG: return "XG";
    case zsibot::Model::MODEL_XGW: return "XGW";
    case zsibot::Model::MODEL_XGWHSPD: return "XGWHSPD";
    default: return "UNKNOWN";
  }
}

}  // 命名空间 genisom_l1_control
