#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iterator>
#include <limits>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include "fast_anchor_fusion/odom_frame_transform.hpp"

namespace fast_anchor_fusion
{

int64_t stampNanoseconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<int64_t>(stamp.sec) * 1000000000LL +
         static_cast<int64_t>(stamp.nanosec);
}

std::array<double, 3> parameterVectorToArray(
  const std::vector<double> & values,
  const std::string & parameter_name)
{
  if (values.size() != 3) {
    throw std::invalid_argument(parameter_name + " must contain exactly 3 values");
  }
  return {values[0], values[1], values[2]};
}

bool isFinitePose(const nav_msgs::msg::Odometry & odometry)
{
  const auto & position = odometry.pose.pose.position;
  const auto & orientation = odometry.pose.pose.orientation;
  return std::isfinite(position.x) && std::isfinite(position.y) && std::isfinite(position.z) &&
         std::isfinite(orientation.x) && std::isfinite(orientation.y) &&
         std::isfinite(orientation.z) && std::isfinite(orientation.w);
}

bool isFiniteTwist(const nav_msgs::msg::Odometry & odometry)
{
  const auto & linear = odometry.twist.twist.linear;
  const auto & angular = odometry.twist.twist.angular;
  return std::isfinite(linear.x) && std::isfinite(linear.y) && std::isfinite(linear.z) &&
         std::isfinite(angular.x) && std::isfinite(angular.y) && std::isfinite(angular.z);
}

void setCovarianceDiagonal(std::array<double, 36> & covariance, const double value)
{
  covariance.fill(0.0);
  for (size_t index = 0; index < 6; ++index) {
    covariance[index * 6 + index] = value;
  }
}

void floorCovarianceDiagonal(std::array<double, 36> & covariance, const double minimum)
{
  for (size_t index = 0; index < 6; ++index) {
    const size_t diagonal_index = index * 6 + index;
    if (!std::isfinite(covariance[diagonal_index]) || covariance[diagonal_index] < minimum) {
      covariance[diagonal_index] = minimum;
    }
  }
}

double yawFromAffine(const Eigen::Affine3d & pose)
{
  const auto rotation = pose.rotation();
  return std::atan2(rotation(1, 0), rotation(0, 0));
}

double normalizeAngle(const double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

class OdomFrameAdapterNode : public rclcpp::Node
{
public:
  OdomFrameAdapterNode()
  : Node("fast_anchor_odom_frame_adapter")
  {
    body_frame_ = declare_parameter<std::string>("frames.body", "body");
    base_frame_ = declare_parameter<std::string>("frames.base", "base_link");
    raw_fast_lio_topic_ =
      declare_parameter<std::string>("topics.raw_fast_lio_odom", "/Odometry");
    fast_lio_base_topic_ = declare_parameter<std::string>(
      "topics.fast_lio_base_odom", "/fast_anchor/fusion/fast_lio_base_odom");
    filtered_base_topic_ = declare_parameter<std::string>(
      "topics.filtered_base_odom", "/fast_anchor/fusion/filtered_base_odom");
    fused_body_topic_ = declare_parameter<std::string>(
      "topics.fused_body_odom", "/fast_anchor/fusion/fused_body_odom");
    leg_odom_topic_ =
      declare_parameter<std::string>("topics.leg_odom", "/leg_odom");

    const auto base_to_body_xyz = parameterVectorToArray(
      declare_parameter<std::vector<double>>("base_to_body_xyz", {0.0, 0.0, 0.0}),
      "base_to_body_xyz");
    const auto base_to_body_rpy = parameterVectorToArray(
      declare_parameter<std::vector<double>>("base_to_body_rpy", {0.0, 0.0, 0.0}),
      "base_to_body_rpy");
    base_to_body_ = makeTransform(base_to_body_xyz, base_to_body_rpy);
    body_to_base_ = base_to_body_.inverse();

    health_check_enabled_ = declare_parameter<bool>("health_check.enabled", true);
    drop_unhealthy_ = declare_parameter<bool>("health_check.drop_unhealthy", false);
    max_translation_step_m_ =
      declare_parameter<double>("health_check.max_translation_step_m", 0.5);
    max_rotation_step_rad_ =
      declare_parameter<double>("health_check.max_rotation_step_rad", 0.6);
    max_linear_speed_mps_ =
      declare_parameter<double>("health_check.max_linear_speed_mps", 2.5);
    max_angular_speed_radps_ =
      declare_parameter<double>("health_check.max_angular_speed_radps", 3.0);
    min_pose_covariance_ =
      declare_parameter<double>("health_check.min_pose_covariance", 1.0e-4);
    min_twist_covariance_ =
      declare_parameter<double>("health_check.min_twist_covariance", 1.0e-3);
    unhealthy_pose_covariance_ =
      declare_parameter<double>("health_check.unhealthy_pose_covariance", 1000.0);
    unhealthy_twist_covariance_ =
      declare_parameter<double>("health_check.unhealthy_twist_covariance", 1000.0);
    min_dt_for_speed_check_s_ =
      declare_parameter<double>("health_check.min_dt_for_speed_check_s", 0.02);
    consistency_check_enabled_ =
      declare_parameter<bool>("consistency_check.enabled", true);
    leg_match_tolerance_s_ =
      declare_parameter<double>("consistency_check.leg_match_tolerance_s", 0.15);
    consistency_min_interval_s_ =
      declare_parameter<double>("consistency_check.min_interval_s", 0.5);
    consistency_max_interval_s_ =
      declare_parameter<double>("consistency_check.max_interval_s", 2.0);
    max_increment_translation_error_m_ =
      declare_parameter<double>("consistency_check.max_translation_error_m", 0.35);
    max_increment_yaw_error_rad_ =
      declare_parameter<double>("consistency_check.max_yaw_error_rad", 0.35);
    stationary_leg_motion_threshold_m_ =
      declare_parameter<double>("consistency_check.stationary_leg_motion_threshold_m", 0.05);
    max_lio_motion_when_leg_stationary_m_ =
      declare_parameter<double>("consistency_check.max_lio_motion_when_leg_stationary_m", 0.18);
    min_distance_for_ratio_check_m_ =
      declare_parameter<double>("consistency_check.min_distance_for_ratio_check_m", 0.08);
    max_increment_distance_ratio_ =
      declare_parameter<double>("consistency_check.max_distance_ratio", 3.0);
    leg_history_size_ =
      declare_parameter<int>("consistency_check.leg_history_size", 200);
    if (leg_history_size_ < 2) {
      throw std::runtime_error("consistency_check.leg_history_size must be at least 2");
    }

    fast_lio_base_pub_ =
      create_publisher<nav_msgs::msg::Odometry>(fast_lio_base_topic_, rclcpp::QoS(50));
    fused_body_pub_ =
      create_publisher<nav_msgs::msg::Odometry>(fused_body_topic_, rclcpp::QoS(50));
    raw_fast_lio_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      raw_fast_lio_topic_, rclcpp::QoS(50),
      std::bind(&OdomFrameAdapterNode::rawFastLioCallback, this, std::placeholders::_1));
    filtered_base_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      filtered_base_topic_, rclcpp::QoS(50),
      std::bind(&OdomFrameAdapterNode::filteredBaseCallback, this, std::placeholders::_1));
    leg_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      leg_odom_topic_, rclcpp::QoS(50),
      std::bind(&OdomFrameAdapterNode::legOdomCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Odometry adapter ready: %s (%s) -> %s (%s), leg=%s, EKF %s (%s) -> %s (%s)",
      raw_fast_lio_topic_.c_str(), body_frame_.c_str(),
      fast_lio_base_topic_.c_str(), base_frame_.c_str(),
      leg_odom_topic_.c_str(),
      filtered_base_topic_.c_str(), base_frame_.c_str(),
      fused_body_topic_.c_str(), body_frame_.c_str());
  }

private:
  void rawFastLioCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (!msg->child_frame_id.empty() && msg->child_frame_id != body_frame_) {
      RCLCPP_WARN_ONCE(
        get_logger(), "Raw FAST-LIO child_frame_id is '%s', expected '%s'.",
        msg->child_frame_id.c_str(), body_frame_.c_str());
    }
    auto output = transformOdometryChildFrame(*msg, body_to_base_, base_frame_);
    const bool healthy = markFastLioHealth(output);
    if (!healthy && drop_unhealthy_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Dropping unhealthy FAST-LIO odometry sample.");
      return;
    }
    fast_lio_base_pub_->publish(output);
  }

  void filteredBaseCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (!msg->child_frame_id.empty() && msg->child_frame_id != base_frame_) {
      RCLCPP_WARN_ONCE(
        get_logger(), "Filtered odometry child_frame_id is '%s', expected '%s'.",
        msg->child_frame_id.c_str(), base_frame_.c_str());
    }
    fused_body_pub_->publish(
      transformOdometryChildFrame(*msg, base_to_body_, body_frame_));
  }

  void legOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (!isFinitePose(*msg)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Ignoring non-finite leg odometry sample.");
      return;
    }
    const int64_t stamp_ns = stampNanoseconds(msg->header.stamp);
    leg_history_[stamp_ns] = poseToAffine(msg->pose.pose);
    while (static_cast<int>(leg_history_.size()) > leg_history_size_) {
      leg_history_.erase(leg_history_.begin());
    }
  }

  bool markFastLioHealth(nav_msgs::msg::Odometry & odometry)
  {
    floorCovarianceDiagonal(odometry.pose.covariance, min_pose_covariance_);
    floorCovarianceDiagonal(odometry.twist.covariance, min_twist_covariance_);
    if (!health_check_enabled_) {
      return true;
    }

    bool healthy = isFinitePose(odometry) && isFiniteTwist(odometry);
    std::string reason;
    const auto current_pose = poseToAffine(odometry.pose.pose);
    const int64_t current_stamp_ns = stampNanoseconds(odometry.header.stamp);
    if (!healthy) {
      reason = "non-finite pose or twist";
    } else if (have_last_fast_lio_) {
      const double dt_s =
        static_cast<double>(current_stamp_ns - last_fast_lio_stamp_ns_) * 1.0e-9;
      const Eigen::Affine3d delta = last_fast_lio_pose_.inverse() * current_pose;
      const double translation_step = delta.translation().norm();
      const double rotation_step = Eigen::AngleAxisd(delta.rotation()).angle();
      if (translation_step > max_translation_step_m_) {
        healthy = false;
        reason = "translation step too large";
      } else if (rotation_step > max_rotation_step_rad_) {
        healthy = false;
        reason = "rotation step too large";
      } else if (dt_s >= min_dt_for_speed_check_s_) {
        const double linear_speed = translation_step / dt_s;
        const double angular_speed = rotation_step / dt_s;
        if (linear_speed > max_linear_speed_mps_) {
          healthy = false;
          reason = "linear speed too large";
        } else if (angular_speed > max_angular_speed_radps_) {
          healthy = false;
          reason = "angular speed too large";
        }
      }
    }
    if (healthy && !markIncrementConsistency(current_stamp_ns, current_pose, reason)) {
      healthy = false;
    }

    if (healthy) {
      last_fast_lio_pose_ = current_pose;
      last_fast_lio_stamp_ns_ = current_stamp_ns;
      have_last_fast_lio_ = true;
      return true;
    }

    // 不健康样本仍可保留时间连续性，但通过大协方差让 EKF 基本只做预测。
    setCovarianceDiagonal(odometry.pose.covariance, unhealthy_pose_covariance_);
    setCovarianceDiagonal(odometry.twist.covariance, unhealthy_twist_covariance_);
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "FAST-LIO odometry marked unhealthy: %s", reason.c_str());
    return false;
  }

  bool markIncrementConsistency(
    const int64_t lio_stamp_ns,
    const Eigen::Affine3d & current_lio_pose,
    std::string & reason)
  {
    if (!consistency_check_enabled_) {
      return true;
    }

    Eigen::Affine3d current_leg_pose;
    if (!findLegPose(lio_stamp_ns, current_leg_pose)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "No leg odometry sample matches FAST-LIO stamp within %.0f ms; skip increment consistency check.",
        leg_match_tolerance_s_ * 1000.0);
      return true;
    }

    if (!have_increment_anchor_) {
      resetIncrementAnchor(lio_stamp_ns, current_lio_pose, current_leg_pose);
      return true;
    }

    const double interval_s =
      static_cast<double>(lio_stamp_ns - anchor_stamp_ns_) * 1.0e-9;
    if (interval_s < 0.0) {
      resetIncrementAnchor(lio_stamp_ns, current_lio_pose, current_leg_pose);
      return true;
    }
    if (interval_s < consistency_min_interval_s_) {
      return true;
    }
    if (interval_s > consistency_max_interval_s_) {
      resetIncrementAnchor(lio_stamp_ns, current_lio_pose, current_leg_pose);
      return true;
    }

    const Eigen::Affine3d lio_delta = anchor_lio_pose_.inverse() * current_lio_pose;
    const Eigen::Affine3d leg_delta = anchor_leg_pose_.inverse() * current_leg_pose;
    const double lio_distance = lio_delta.translation().norm();
    const double leg_distance = leg_delta.translation().norm();
    const double translation_error = std::abs(lio_distance - leg_distance);
    const double yaw_error =
      std::abs(normalizeAngle(yawFromAffine(lio_delta) - yawFromAffine(leg_delta)));

    resetIncrementAnchor(lio_stamp_ns, current_lio_pose, current_leg_pose);

    // Unitree 认为基本静止时，FAST-LIO 不能在同一时间窗内出现明显位移。
    if (leg_distance < stationary_leg_motion_threshold_m_ &&
      lio_distance > max_lio_motion_when_leg_stationary_m_)
    {
      reason = "FAST-LIO moves while leg odometry is stationary";
      return false;
    }
    if (translation_error > max_increment_translation_error_m_) {
      reason = "FAST-LIO and leg odometry translation increments diverge";
      return false;
    }
    if (lio_distance > min_distance_for_ratio_check_m_ &&
      leg_distance > min_distance_for_ratio_check_m_)
    {
      const double distance_ratio = std::max(lio_distance, leg_distance) /
        std::max(std::min(lio_distance, leg_distance), 1.0e-6);
      if (distance_ratio > max_increment_distance_ratio_) {
        reason = "FAST-LIO and leg odometry distance ratio diverges";
        return false;
      }
    }
    if (yaw_error > max_increment_yaw_error_rad_) {
      reason = "FAST-LIO and leg odometry yaw increments diverge";
      return false;
    }
    return true;
  }

  bool findLegPose(
    const int64_t stamp_ns,
    Eigen::Affine3d & pose) const
  {
    if (leg_history_.empty()) {
      return false;
    }

    auto candidate = leg_history_.lower_bound(stamp_ns);
    auto best = leg_history_.end();
    int64_t best_abs_delta_ns = std::numeric_limits<int64_t>::max();
    if (candidate != leg_history_.end()) {
      best = candidate;
      best_abs_delta_ns = std::abs(candidate->first - stamp_ns);
    }
    if (candidate != leg_history_.begin()) {
      const auto previous = std::prev(candidate);
      const int64_t previous_abs_delta_ns = std::abs(previous->first - stamp_ns);
      if (previous_abs_delta_ns < best_abs_delta_ns) {
        best = previous;
        best_abs_delta_ns = previous_abs_delta_ns;
      }
    }
    if (best == leg_history_.end() ||
      static_cast<double>(best_abs_delta_ns) * 1.0e-9 > leg_match_tolerance_s_)
    {
      return false;
    }

    pose = best->second;
    return true;
  }

  void resetIncrementAnchor(
    const int64_t stamp_ns,
    const Eigen::Affine3d & lio_pose,
    const Eigen::Affine3d & leg_pose)
  {
    anchor_stamp_ns_ = stamp_ns;
    anchor_lio_pose_ = lio_pose;
    anchor_leg_pose_ = leg_pose;
    have_increment_anchor_ = true;
  }

  std::string body_frame_;
  std::string base_frame_;
  std::string raw_fast_lio_topic_;
  std::string fast_lio_base_topic_;
  std::string filtered_base_topic_;
  std::string fused_body_topic_;
  std::string leg_odom_topic_;
  bool health_check_enabled_ = true;
  bool drop_unhealthy_ = false;
  bool have_last_fast_lio_ = false;
  bool consistency_check_enabled_ = true;
  bool have_increment_anchor_ = false;
  int64_t last_fast_lio_stamp_ns_ = 0;
  int64_t anchor_stamp_ns_ = 0;
  int leg_history_size_ = 200;
  double max_translation_step_m_ = 0.5;
  double max_rotation_step_rad_ = 0.6;
  double max_linear_speed_mps_ = 2.5;
  double max_angular_speed_radps_ = 3.0;
  double min_pose_covariance_ = 1.0e-4;
  double min_twist_covariance_ = 1.0e-3;
  double unhealthy_pose_covariance_ = 1000.0;
  double unhealthy_twist_covariance_ = 1000.0;
  double min_dt_for_speed_check_s_ = 0.02;
  double leg_match_tolerance_s_ = 0.15;
  double consistency_min_interval_s_ = 0.5;
  double consistency_max_interval_s_ = 2.0;
  double max_increment_translation_error_m_ = 0.35;
  double max_increment_yaw_error_rad_ = 0.35;
  double stationary_leg_motion_threshold_m_ = 0.05;
  double max_lio_motion_when_leg_stationary_m_ = 0.18;
  double min_distance_for_ratio_check_m_ = 0.08;
  double max_increment_distance_ratio_ = 3.0;
  Eigen::Affine3d base_to_body_ = Eigen::Affine3d::Identity();
  Eigen::Affine3d body_to_base_ = Eigen::Affine3d::Identity();
  Eigen::Affine3d last_fast_lio_pose_ = Eigen::Affine3d::Identity();
  Eigen::Affine3d anchor_lio_pose_ = Eigen::Affine3d::Identity();
  Eigen::Affine3d anchor_leg_pose_ = Eigen::Affine3d::Identity();
  std::map<int64_t, Eigen::Affine3d> leg_history_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr raw_fast_lio_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr filtered_base_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr leg_odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr fast_lio_base_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr fused_body_pub_;
};

}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<fast_anchor_fusion::OdomFrameAdapterNode>());
  rclcpp::shutdown();
  return 0;
}
