#ifndef GENISOM_L1_CONTROL__SDK_WRAPPER_HPP_
#define GENISOM_L1_CONTROL__SDK_WRAPPER_HPP_

#include <array>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "zsibot_sdk/zsibot_api.h"

namespace genisom_l1_control
{

struct NormalizedCommand
{
  float forward{0.0F};
  float yaw{0.0F};
  float lateral{0.0F};
};

struct CommandLimits
{
  bool limit_cmd_vel_input{false};
  bool exclusive_translation_rotation{true};
  double max_linear_x{0.10};
  double max_linear_y{0.05};
  double max_angular_z{0.20};
  double forward_joystick_per_mps{1.0};
  double lateral_joystick_per_mps{2.0};
  double yaw_joystick_per_rps{0.5};
  double max_forward_joystick{1.0};
  double max_lateral_joystick{1.0};
  double max_yaw_joystick{1.0};
  double linear_deadband{0.01};
  double angular_deadband{0.05};
};

struct RobotSnapshot
{
  bool connected{false};
  std::uint32_t power{0};
  float temperature{0.0F};
  std::string serial_number;
  std::string device_name;
  std::string wifi_ssid;
  zsibot::VersionInfo version;
  zsibot::SpeedInfo speed;
  zsibot::BatteryInfo battery;
  zsibot::Model model{zsibot::Model::MODEL_XG};
  zsibot::SpeedLevel speed_level{zsibot::SpeedLevel::SL_NULL};
  zsibot::FunctionMode function_mode{zsibot::FunctionMode::FM_NULL};
  zsibot::ControlMode control_mode{zsibot::ControlMode::CM_NULL};
  zsibot::MotionMode motion_mode{zsibot::MotionMode::MM_NULL};
  zsibot::MotionType motion_type{zsibot::MotionType::MT_NULL};
  std::array<float, 4> quaternion{};
  std::array<float, 3> rpy{};
  std::array<float, 3> body_acceleration{};
  std::array<float, 3> body_gyro{};
  std::array<float, 3> position{};
  std::array<float, 3> world_velocity{};
  std::array<float, 3> body_velocity{};
  std::array<float, 16> motor_temperatures{};
  std::array<std::array<float, 4>, 4> joint_position{};
  std::array<std::array<float, 4>, 4> joint_velocity{};
  std::array<std::array<float, 4>, 4> joint_effort{};
  std::vector<zsibot::FaultInfo> faults;
};

NormalizedCommand convert_twist_to_normalized(
  double linear_x, double linear_y, double angular_z, const CommandLimits & limits);
NormalizedCommand convert_twist_to_longitudinal(
  double linear_x, double angular_z, const CommandLimits & limits);
bool command_supported_for_model(zsibot::CmdCode command, zsibot::Model model);
bool command_requires_lab_mode(zsibot::CmdCode command);
bool velocity_bridge_may_send(
  bool connected, bool bridge_enabled, bool estop_latched, bool transition_pending,
  zsibot::FunctionMode owner);
bool sdk_control_lost(bool sdk_control_confirmed, zsibot::FunctionMode owner);

class SdkWrapper
{
public:
  SdkWrapper(const std::string & robot_ip, std::uint32_t send_port, std::uint32_t recv_port);
  ~SdkWrapper();

  SdkWrapper(const SdkWrapper &) = delete;
  SdkWrapper & operator=(const SdkWrapper &) = delete;

  bool is_connected() const;
  zsibot::FunctionMode function_mode() const;
  zsibot::ControlMode control_mode() const;
  zsibot::MotionMode motion_mode() const;
  zsibot::Model model() const;
  RobotSnapshot read_snapshot() const;

  void request_control(zsibot::FunctionMode target);
  void send_command(zsibot::CmdCode command);
  void send_normalized(const NormalizedCommand & command);
  void stop_motion();

private:
  mutable std::mutex mutex_;
  std::unique_ptr<zsibot::ZsibotExecutor> executor_;
};

std::string function_mode_name(zsibot::FunctionMode mode);
std::string control_mode_name(zsibot::ControlMode mode);
std::string speed_level_name(zsibot::SpeedLevel level);
std::string motion_mode_name(zsibot::MotionMode mode);
std::string motion_type_name(zsibot::MotionType type);
std::string model_name(zsibot::Model model);

}  // 命名空间 genisom_l1_control

#endif
