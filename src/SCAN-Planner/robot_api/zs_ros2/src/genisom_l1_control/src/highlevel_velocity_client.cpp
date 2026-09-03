#include "genisom_l1_control/highlevel_velocity_client.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace genisom_l1_control
{
namespace
{

float velocity_axis(double value, bool limit_input, double input_limit, double deadband)
{
  if (!std::isfinite(value)) {
    return 0.0F;
  }
  const double filtered = std::abs(value) <= deadband ? 0.0 : value;
  const double limited = limit_input ? std::clamp(filtered, -input_limit, input_limit) : filtered;
  return static_cast<float>(limited);
}

}  // 匿名命名空间

VelocityCommand convert_twist_to_velocity(
  double linear_x, double linear_y, double angular_z, const VelocityLimits & limits)
{
  if (limits.limit_cmd_vel_input &&
    (!std::isfinite(limits.max_linear_x) || !std::isfinite(limits.max_linear_y) ||
    !std::isfinite(limits.max_angular_z) || limits.max_linear_x <= 0.0 ||
    limits.max_linear_y <= 0.0 || limits.max_angular_z <= 0.0))
  {
    throw std::invalid_argument("启用 cmd_vel 输入限幅时，物理速度上限必须大于 0");
  }
  if (!std::isfinite(limits.linear_deadband) || !std::isfinite(limits.angular_deadband) ||
    limits.linear_deadband < 0.0 || limits.angular_deadband < 0.0)
  {
    throw std::invalid_argument("速度死区必须是有限非负数");
  }

  return VelocityCommand{
    velocity_axis(
      linear_x, limits.limit_cmd_vel_input, limits.max_linear_x,
      limits.linear_deadband),
    velocity_axis(
      linear_y, limits.limit_cmd_vel_input, limits.max_linear_y,
      limits.linear_deadband),
    velocity_axis(
      angular_z, limits.limit_cmd_vel_input, limits.max_angular_z, limits.angular_deadband)};
}

HighLevelVelocityClient::HighLevelVelocityClient(
  const std::string & local_ip, int local_port, const std::string & robot_ip)
{
  highlevel_.initRobot(local_ip, local_port, robot_ip);
}

HighLevelVelocityClient::~HighLevelVelocityClient() = default;

bool HighLevelVelocityClient::is_connected()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return highlevel_.checkConnect();
}

std::uint32_t HighLevelVelocityClient::control_mode()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return highlevel_.getCurrentCtrlmode();
}

void HighLevelVelocityClient::stand_up()
{
  std::lock_guard<std::mutex> lock(mutex_);
  throw_on_error(highlevel_.standUp(), "HighLevel::standUp");
}

void HighLevelVelocityClient::send_velocity(const VelocityCommand & command)
{
  std::lock_guard<std::mutex> lock(mutex_);
  throw_on_error(
    highlevel_.move(command.linear_x, command.linear_y, command.angular_z),
    "HighLevel::move");
}

void HighLevelVelocityClient::stop_motion()
{
  send_velocity(VelocityCommand{});
}

void HighLevelVelocityClient::throw_on_error(std::uint32_t code, const std::string & action)
{
  if (code != 0U) {
    throw std::runtime_error(action + " 返回错误码 " + std::to_string(code));
  }
}

std::string highlevel_control_mode_name(std::uint32_t mode)
{
  switch (mode) {
    case 0U: return "PASSIVE";
    case 1U: return "STAND";
    case 3U: return "MOVE";
    default: return "UNKNOWN(" + std::to_string(mode) + ")";
  }
}

}  // 命名空间 genisom_l1_control
