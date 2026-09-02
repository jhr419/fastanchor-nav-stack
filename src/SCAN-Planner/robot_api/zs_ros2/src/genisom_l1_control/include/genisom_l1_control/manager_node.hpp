#ifndef GENISOM_L1_CONTROL__MANAGER_NODE_HPP_
#define GENISOM_L1_CONTROL__MANAGER_NODE_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "genisom_l1_control/sdk_wrapper.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace genisom_l1_control
{

class ManagerNode : public rclcpp::Node
{
public:
  explicit ManagerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~ManagerNode() override;

private:
  enum class CommandKind
  {
    MODE,
    SPEED,
    ACTION
  };

  using Trigger = std_srvs::srv::Trigger;
  using SetBool = std_srvs::srv::SetBool;
  using SteadyTime = std::chrono::steady_clock::time_point;
  using TriggerCallback = std::function<void (
        const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response>)>;

  void create_publishers();
  void create_services();
  void create_timers();
  void add_trigger_service(const std::string & name, TriggerCallback callback);
  void add_command_service(
    const std::string & name, zsibot::CmdCode command, const std::string & label,
    CommandKind kind);

  void on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr message);
  void on_request_control(
    zsibot::FunctionMode target, bool clears_estop,
    const std::shared_ptr<Trigger::Request> request,
    std::shared_ptr<Trigger::Response> response);
  void on_sdk_command(
    zsibot::CmdCode command, const std::string & label, CommandKind kind,
    const std::shared_ptr<Trigger::Request> request,
    std::shared_ptr<Trigger::Response> response);
  void on_set_velocity_bridge(
    const std::shared_ptr<SetBool::Request> request,
    std::shared_ptr<SetBool::Response> response);
  void on_emergency_stop(
    const std::shared_ptr<Trigger::Request> request,
    std::shared_ptr<Trigger::Response> response);

  void on_control_timer();
  void on_telemetry_timer();
  void on_status_timer();
  void handle_connection(bool connected, SteadyTime now);
  void handle_owner_feedback(zsibot::FunctionMode owner, SteadyTime now);
  void handle_cmd_vel_watchdog(SteadyTime now);
  void disable_velocity_bridge(bool send_stop, const std::string & reason);
  void invalidate_cmd_vel();

  void publish_telemetry(const RobotSnapshot & snapshot, const rclcpp::Time & stamp);
  void publish_imu(const RobotSnapshot & snapshot, const rclcpp::Time & stamp);
  void publish_odom(const RobotSnapshot & snapshot, const rclcpp::Time & stamp);
  void publish_joint_states(const RobotSnapshot & snapshot, const rclcpp::Time & stamp);
  void publish_battery(const RobotSnapshot & snapshot, const rclcpp::Time & stamp);
  void publish_velocity(const RobotSnapshot & snapshot, const rclcpp::Time & stamp);
  void publish_motor_temperatures(const RobotSnapshot & snapshot);
  void publish_device_info(const RobotSnapshot & snapshot);
  diagnostic_msgs::msg::DiagnosticStatus build_manager_status(SteadyTime now) const;
  diagnostic_msgs::msg::DiagnosticArray build_fault_diagnostics(const rclcpp::Time & stamp) const;
  void publish_connection(bool connected);

  mutable std::mutex mutex_;
  std::unique_ptr<SdkWrapper> sdk_;
  CommandLimits limits_;
  std::optional<RobotSnapshot> latest_snapshot_;
  std::optional<zsibot::FunctionMode> pending_owner_;
  SteadyTime owner_transition_deadline_{};
  SteadyTime last_cmd_vel_time_{};
  zsibot::FunctionMode last_owner_{zsibot::FunctionMode::FM_NULL};
  bool pending_owner_clears_estop_{false};
  bool connected_{false};
  bool velocity_bridge_enabled_{false};
  bool have_fresh_cmd_vel_{false};
  bool watchdog_stop_sent_{false};
  bool estop_latched_{false};
  std::string last_command_;
  std::string last_error_;

  int cmd_vel_timeout_ms_{300};
  int mode_switch_timeout_ms_{2000};
  double telemetry_rate_hz_{20.0};
  double status_rate_hz_{5.0};
  std::string imu_frame_id_;
  std::string odom_frame_id_;
  std::string base_frame_id_;
  bool publish_imu_enabled_{true};
  bool publish_odom_enabled_{true};
  bool publish_joint_states_enabled_{true};
  bool publish_battery_enabled_{true};
  bool publish_velocity_enabled_{true};
  bool publish_motor_temperatures_enabled_{true};
  bool publish_diagnostics_enabled_{true};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr motor_temperatures_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr model_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr version_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr connection_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr faults_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  std::vector<rclcpp::Service<Trigger>::SharedPtr> trigger_services_;
  rclcpp::Service<SetBool>::SharedPtr velocity_bridge_service_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr telemetry_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

}  // 命名空间 genisom_l1_control

#endif
