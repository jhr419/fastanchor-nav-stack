#ifndef FAST_ANCHOR_FUSION__ODOM_FRAME_TRANSFORM_HPP_
#define FAST_ANCHOR_FUSION__ODOM_FRAME_TRANSFORM_HPP_

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>
#include <string>

#include <nav_msgs/msg/odometry.hpp>

namespace fast_anchor_fusion
{

using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Matrix6dRowMajor = Eigen::Matrix<double, 6, 6, Eigen::RowMajor>;

inline Eigen::Matrix3d skew(const Eigen::Vector3d & vector)
{
  Eigen::Matrix3d result;
  result <<
    0.0, -vector.z(), vector.y(),
    vector.z(), 0.0, -vector.x(),
    -vector.y(), vector.x(), 0.0;
  return result;
}

inline Eigen::Affine3d poseToAffine(const geometry_msgs::msg::Pose & pose)
{
  Eigen::Quaterniond rotation(
    pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  if (!std::isfinite(rotation.norm()) || rotation.norm() < 1e-12) {
    rotation = Eigen::Quaterniond::Identity();
  } else {
    rotation.normalize();
  }
  return Eigen::Translation3d(
    pose.position.x, pose.position.y, pose.position.z) * rotation;
}

inline geometry_msgs::msg::Pose affineToPose(const Eigen::Affine3d & affine)
{
  geometry_msgs::msg::Pose pose;
  const Eigen::Vector3d translation = affine.translation();
  const Eigen::Quaterniond rotation(affine.rotation());
  pose.position.x = translation.x();
  pose.position.y = translation.y();
  pose.position.z = translation.z();
  pose.orientation.x = rotation.x();
  pose.orientation.y = rotation.y();
  pose.orientation.z = rotation.z();
  pose.orientation.w = rotation.w();
  return pose;
}

inline Eigen::Affine3d makeTransform(
  const std::array<double, 3> & xyz,
  const std::array<double, 3> & rpy)
{
  return Eigen::Translation3d(xyz[0], xyz[1], xyz[2]) *
         Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX());
}

inline void copyCovariance(
  const Matrix6d & covariance,
  std::array<double, 36> & destination)
{
  const Matrix6dRowMajor row_major = covariance;
  std::copy(row_major.data(), row_major.data() + destination.size(), destination.begin());
}

inline nav_msgs::msg::Odometry transformOdometryChildFrame(
  const nav_msgs::msg::Odometry & input,
  const Eigen::Affine3d & source_to_target,
  const std::string & target_child_frame)
{
  nav_msgs::msg::Odometry output = input;
  const Eigen::Affine3d world_to_source = poseToAffine(input.pose.pose);
  const Eigen::Affine3d world_to_target = world_to_source * source_to_target;
  output.pose.pose = affineToPose(world_to_target);
  output.child_frame_id = target_child_frame;

  // 位姿协方差使用父坐标系固定轴，杆臂会把姿态不确定性耦合到目标位置。
  const Eigen::Map<const Matrix6dRowMajor> pose_covariance(input.pose.covariance.data());
  Matrix6d pose_jacobian = Matrix6d::Identity();
  pose_jacobian.block<3, 3>(0, 3) =
    -world_to_source.rotation() * skew(source_to_target.translation());
  Matrix6d transformed_pose_covariance =
    pose_jacobian * pose_covariance * pose_jacobian.transpose();
  transformed_pose_covariance =
    0.5 * (transformed_pose_covariance + transformed_pose_covariance.transpose());
  copyCovariance(transformed_pose_covariance, output.pose.covariance);

  // Odometry 的 twist 位于 child_frame_id，下式同时完成杆臂平移和坐标轴旋转。
  const Eigen::Matrix3d target_from_source = source_to_target.rotation().transpose();
  Matrix6d twist_jacobian = Matrix6d::Zero();
  twist_jacobian.block<3, 3>(0, 0) = target_from_source;
  twist_jacobian.block<3, 3>(0, 3) =
    -target_from_source * skew(source_to_target.translation());
  twist_jacobian.block<3, 3>(3, 3) = target_from_source;

  Eigen::Matrix<double, 6, 1> source_twist;
  source_twist <<
    input.twist.twist.linear.x,
    input.twist.twist.linear.y,
    input.twist.twist.linear.z,
    input.twist.twist.angular.x,
    input.twist.twist.angular.y,
    input.twist.twist.angular.z;
  const Eigen::Matrix<double, 6, 1> target_twist = twist_jacobian * source_twist;
  output.twist.twist.linear.x = target_twist(0);
  output.twist.twist.linear.y = target_twist(1);
  output.twist.twist.linear.z = target_twist(2);
  output.twist.twist.angular.x = target_twist(3);
  output.twist.twist.angular.y = target_twist(4);
  output.twist.twist.angular.z = target_twist(5);

  const Eigen::Map<const Matrix6dRowMajor> twist_covariance(input.twist.covariance.data());
  Matrix6d transformed_twist_covariance =
    twist_jacobian * twist_covariance * twist_jacobian.transpose();
  transformed_twist_covariance =
    0.5 * (transformed_twist_covariance + transformed_twist_covariance.transpose());
  copyCovariance(transformed_twist_covariance, output.twist.covariance);
  return output;
}

}

#endif
