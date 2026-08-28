#include <array>
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

std::array<double, 3> parameterVectorToArray(
  const std::vector<double> & values,
  const std::string & parameter_name)
{
  if (values.size() != 3) {
    throw std::invalid_argument(parameter_name + " must contain exactly 3 values");
  }
  return {values[0], values[1], values[2]};
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
    fast_lio_base_pub_->publish(
      transformOdometryChildFrame(*msg, body_to_base_, base_frame_));
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

  std::string body_frame_;
  std::string base_frame_;
  std::string raw_fast_lio_topic_;
  std::string fast_lio_base_topic_;
  std::string filtered_base_topic_;
  std::string fused_body_topic_;
  Eigen::Affine3d base_to_body_ = Eigen::Affine3d::Identity();
  Eigen::Affine3d body_to_base_ = Eigen::Affine3d::Identity();
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
