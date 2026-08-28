#include <gtest/gtest.h>

#include <cmath>
#include <string>

#include "fast_anchor_fusion/odom_frame_transform.hpp"

namespace fast_anchor_fusion
{
namespace
{

nav_msgs::msg::Odometry makeOdometry()
{
  nav_msgs::msg::Odometry odometry;
  odometry.header.frame_id = "camera_init";
  odometry.child_frame_id = "body";
  odometry.pose.pose.orientation.w = 1.0;
  for (size_t i = 0; i < 6; ++i) {
    odometry.pose.covariance[i * 6 + i] = 0.1 + static_cast<double>(i) * 0.01;
    odometry.twist.covariance[i * 6 + i] = 0.2 + static_cast<double>(i) * 0.01;
  }
  return odometry;
}

}

TEST(OdomFrameTransform, AppliesLeverArmInWorldFrame)
{
  constexpr double quarter_turn_half_angle = 0.7853981633974483;
  auto input = makeOdometry();
  input.pose.pose.position.x = 1.0;
  input.pose.pose.orientation.z = std::sin(quarter_turn_half_angle);
  input.pose.pose.orientation.w = std::cos(quarter_turn_half_angle);

  const Eigen::Affine3d body_to_base =
    makeTransform({0.3, 0.0, 0.0}, {0.0, 0.0, 0.0});
  const auto output = transformOdometryChildFrame(input, body_to_base, "base_link");

  EXPECT_NEAR(output.pose.pose.position.x, 1.0, 1e-9);
  EXPECT_NEAR(output.pose.pose.position.y, 0.3, 1e-9);
  EXPECT_NEAR(output.pose.pose.position.z, 0.0, 1e-9);
  EXPECT_EQ(output.header.frame_id, "camera_init");
  EXPECT_EQ(output.child_frame_id, "base_link");
}

TEST(OdomFrameTransform, ShiftsTwistToTargetOrigin)
{
  auto input = makeOdometry();
  input.twist.twist.angular.z = 2.0;

  const Eigen::Affine3d source_to_target =
    makeTransform({0.5, 0.0, 0.0}, {0.0, 0.0, 0.0});
  const auto output = transformOdometryChildFrame(input, source_to_target, "target");

  EXPECT_NEAR(output.twist.twist.linear.x, 0.0, 1e-9);
  EXPECT_NEAR(output.twist.twist.linear.y, 1.0, 1e-9);
  EXPECT_NEAR(output.twist.twist.angular.z, 2.0, 1e-9);
}

TEST(OdomFrameTransform, PoseRoundTripRecoversInput)
{
  auto input = makeOdometry();
  input.pose.pose.position.x = 2.0;
  input.pose.pose.position.y = -1.0;
  input.pose.pose.position.z = 0.5;
  const Eigen::Quaterniond rotation(
    Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(-0.2, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitX()));
  input.pose.pose.orientation.x = rotation.x();
  input.pose.pose.orientation.y = rotation.y();
  input.pose.pose.orientation.z = rotation.z();
  input.pose.pose.orientation.w = rotation.w();

  const Eigen::Affine3d source_to_target =
    makeTransform({0.3, -0.1, 0.2}, {0.05, -0.1, 0.2});
  const auto transformed = transformOdometryChildFrame(input, source_to_target, "target");
  const auto recovered =
    transformOdometryChildFrame(transformed, source_to_target.inverse(), "body");

  const Eigen::Affine3d expected = poseToAffine(input.pose.pose);
  const Eigen::Affine3d actual = poseToAffine(recovered.pose.pose);
  EXPECT_TRUE(expected.matrix().isApprox(actual.matrix(), 1e-9));
}

}
