#ifndef GENISOM_L1_CONTROL__TWIST_NODE_HPP_
#define GENISOM_L1_CONTROL__TWIST_NODE_HPP_

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "genisom_l1_control/sdk_wrapper.hpp"
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
    WAIT_SDK_CONTROL,
    WAIT_STAND,
    WAIT_MOVE_MODE,
    ACTIVE,
    SHUTTING_DOWN
  };

  void on_control_timer();
  void on_status_timer();
  void on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr message);

  void enter_wait_sdk_control(SteadyTime now);
  void handle_wait_sdk_control(SteadyTime now, zsibot::FunctionMode owner);
  void begin_motion_mode_sequence(SteadyTime now, zsibot::ControlMode control_mode);
  void handle_wait_stand(SteadyTime now, zsibot::ControlMode control_mode);
  void handle_wait_move_mode(SteadyTime now, zsibot::ControlMode control_mode);
  void activate_velocity_control();
  void handle_cmd_vel_watchdog(SteadyTime now);
  void request_shutdown(const std::string & reason, bool release_remote);
  void publish_ready(bool ready);
  void publish_status();

  static std::string state_name(State state);

  std::unique_ptr<SdkWrapper> sdk_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;

  CommandLimits limits_;
  State state_{State::WAIT_CONNECTION};
  bool connected_{false};
  bool connected_once_{false};
  bool sdk_control_confirmed_{false};
  bool ready_{false};
  bool have_cmd_vel_{false};
  bool watchdog_stop_sent_{false};
  bool shutdown_requested_{false};
  bool auto_stand_{true};
  bool auto_move_mode_{true};
  bool exit_on_control_loss_{true};
  int cmd_vel_timeout_ms_{300};
  int connection_timeout_ms_{10000};
  int disconnect_grace_ms_{10000};
  int control_request_timeout_ms_{5000};
  int mode_command_timeout_ms_{8000};
  int command_retry_ms_{500};
  double status_rate_hz_{5.0};

  zsibot::FunctionMode last_owner_{zsibot::FunctionMode::FM_NULL};
  zsibot::ControlMode last_control_mode_{zsibot::ControlMode::CM_NULL};
  SteadyTime node_started_at_{};
  SteadyTime state_deadline_{};
  SteadyTime last_command_sent_at_{};
  SteadyTime last_cmd_vel_at_{};
  std::optional<SteadyTime> disconnected_since_;
  NormalizedCommand active_command_;
  std::string last_error_;
};

}  // 命名空间 genisom_l1_control

#endif
