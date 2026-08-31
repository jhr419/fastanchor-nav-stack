#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
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

    RCLCPP_INFO(
      get_logger(),
      "Odometry adapter ready: %s (%s) -> %s (%s), EKF %s (%s) -> %s (%s)",
      raw_fast_lio_topic_.c_str(), body_frame_.c_str(),
      fast_lio_base_topic_.c_str(), base_frame_.c_str(),
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

  std::string body_frame_;
  std::string base_frame_;
  std::string raw_fast_lio_topic_;
  std::string fast_lio_base_topic_;
  std::string filtered_base_topic_;
  std::string fused_body_topic_;
  bool health_check_enabled_ = true;
  bool drop_unhealthy_ = false;
  bool have_last_fast_lio_ = false;
  int64_t last_fast_lio_stamp_ns_ = 0;
  double max_translation_step_m_ = 0.5;
  double max_rotation_step_rad_ = 0.6;
  double max_linear_speed_mps_ = 2.5;
  double max_angular_speed_radps_ = 3.0;
  double min_pose_covariance_ = 1.0e-4;
  double min_twist_covariance_ = 1.0e-3;
  double unhealthy_pose_covariance_ = 1000.0;
  double unhealthy_twist_covariance_ = 1000.0;
  double min_dt_for_speed_check_s_ = 0.02;
  Eigen::Affine3d base_to_body_ = Eigen::Affine3d::Identity();
  Eigen::Affine3d body_to_base_ = Eigen::Affine3d::Identity();
  Eigen::Affine3d last_fast_lio_pose_ = Eigen::Affine3d::Identity();
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr raw_fast_lio_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr filtered_base_sub_;
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
