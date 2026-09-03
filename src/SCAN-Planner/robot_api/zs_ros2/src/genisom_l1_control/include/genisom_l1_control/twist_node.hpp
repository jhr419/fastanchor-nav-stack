#ifndef GENISOM_L1_CONTROL__TWIST_NODE_HPP_
#define GENISOM_L1_CONTROL__TWIST_NODE_HPP_

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "genisom_l1_control/highlevel_velocity_client.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

namespace genisom_l1_control
{

class TwistNode : public rclcpp::Node
{
public:
  explicit TwistNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~TwistNode() override;

private:
  using SteadyTime = std::chrono::steady_clock::time_point;

  enum class State
  {
    WAIT_CONNECTION,
    WAIT_STAND,
    ACTIVE,
    SHUTTING_DOWN
  };

  void on_control_timer();
  void on_status_timer();
  void on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr message);

  void handle_wait_connection(SteadyTime now);
  void begin_stand_sequence(SteadyTime now);
  void handle_wait_stand(SteadyTime now, std::uint32_t control_mode);
  void activate_velocity_control();
  void request_shutdown(const std::string & reason, bool send_stop);
  void publish_ready(bool ready);
  void publish_status();

  static std::string state_name(State state);

  std::unique_ptr<HighLevelVelocityClient> velocity_client_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;

  VelocityLimits limits_;
  State state_{State::WAIT_CONNECTION};
  bool connected_{false};
  bool connected_once_{false};
  bool ready_{false};
  bool have_cmd_vel_{false};
  bool shutdown_requested_{false};
  bool auto_stand_{true};
  int connection_timeout_ms_{10000};
  int mode_command_timeout_ms_{8000};
  int command_retry_ms_{500};
  int local_port_{43988};
  double status_rate_hz_{5.0};
  std::string local_ip_;
  std::string robot_ip_;

  std::uint32_t last_control_mode_{0U};
  SteadyTime node_started_at_{};
  SteadyTime state_deadline_{};
  SteadyTime last_command_sent_at_{};
  SteadyTime last_cmd_vel_at_{};
  VelocityCommand last_velocity_command_;
  std::string last_error_;
};

}  // 命名空间 genisom_l1_control

#endif
