#ifndef GENISOM_L1_CONTROL__HIGHLEVEL_VELOCITY_CLIENT_HPP_
#define GENISOM_L1_CONTROL__HIGHLEVEL_VELOCITY_CLIENT_HPP_

#include <cstdint>
#include <mutex>
#include <string>

#include "zsl-1w/highlevel.h"

namespace genisom_l1_control
{

struct VelocityCommand
{
  float linear_x{0.0F};
  float linear_y{0.0F};
  float angular_z{0.0F};
};

struct VelocityLimits
{
  bool limit_cmd_vel_input{false};
  double max_linear_x{3.7};
  double max_linear_y{1.0};
  double max_angular_z{3.0};
  double linear_deadband{0.0};
  double angular_deadband{0.0};
};

VelocityCommand convert_twist_to_velocity(
  double linear_x, double linear_y, double angular_z, const VelocityLimits & limits);

class HighLevelVelocityClient
{
public:
  HighLevelVelocityClient(
    const std::string & local_ip, int local_port, const std::string & robot_ip);
  ~HighLevelVelocityClient();

  HighLevelVelocityClient(const HighLevelVelocityClient &) = delete;
  HighLevelVelocityClient & operator=(const HighLevelVelocityClient &) = delete;

  bool is_connected();
  std::uint32_t control_mode();
  void stand_up();
  void send_velocity(const VelocityCommand & command);
  void stop_motion();

private:
  static void throw_on_error(std::uint32_t code, const std::string & action);

  mutable std::mutex mutex_;
  mc_sdk::zsl_1w::HighLevel highlevel_;
};

std::string highlevel_control_mode_name(std::uint32_t mode);

}  // 命名空间 genisom_l1_control

#endif
