#include <gtest/gtest.h>

#include <stdexcept>

#include "plan_manage/motion_constraints.hpp"

namespace scan_planner
{
TEST(MotionConstraintsTest, NonholonomicModeRemovesLateralVelocity)
{
  MotionConstraints constraints;
  constraints.motion_model = MotionModel::kNonholonomic;
  constraints.forward_only = true;
  constraints.max_vx = 0.75;
  constraints.max_vy = 0.35;
  constraints.max_vyaw = 1.0;

  const PlanarCommand command = constrainPlanarCommand(0.5, 0.3, 0.4, constraints);

  EXPECT_DOUBLE_EQ(command.vx, 0.5);
  EXPECT_DOUBLE_EQ(command.vy, 0.0);
  EXPECT_DOUBLE_EQ(command.vyaw, 0.4);
}

TEST(MotionConstraintsTest, ForwardOnlyModeRejectsReverseVelocity)
{
  MotionConstraints constraints;
  constraints.motion_model = MotionModel::kNonholonomic;
  constraints.forward_only = true;
  constraints.max_vx = 0.75;
  constraints.max_vyaw = 1.0;

  const PlanarCommand command = constrainPlanarCommand(-0.5, 0.0, 0.0, constraints);

  EXPECT_DOUBLE_EQ(command.vx, 0.0);
}

TEST(MotionConstraintsTest, YawPrioritySuppressesTranslation)
{
  MotionConstraints constraints;
  constraints.motion_model = MotionModel::kNonholonomic;
  constraints.forward_only = false;
  constraints.exclusive_translation_rotation = true;
  constraints.max_vx = 0.75;
  constraints.max_vyaw = 1.0;
  constraints.linear_deadband = 0.01;
  constraints.angular_deadband = 0.05;

  const PlanarCommand command = constrainPlanarCommand(0.5, 0.0, 0.4, constraints);

  EXPECT_DOUBLE_EQ(command.vx, 0.0);
  EXPECT_DOUBLE_EQ(command.vy, 0.0);
  EXPECT_DOUBLE_EQ(command.vyaw, 0.4);
}

TEST(MotionConstraintsTest, ReverseTranslationRunsAfterYawEntersDeadband)
{
  MotionConstraints constraints;
  constraints.motion_model = MotionModel::kNonholonomic;
  constraints.forward_only = false;
  constraints.exclusive_translation_rotation = true;
  constraints.max_vx = 0.75;
  constraints.max_vyaw = 1.0;
  constraints.linear_deadband = 0.01;
  constraints.angular_deadband = 0.05;

  const PlanarCommand command = constrainPlanarCommand(-0.5, 0.0, 0.04, constraints);

  EXPECT_DOUBLE_EQ(command.vx, -0.5);
  EXPECT_DOUBLE_EQ(command.vy, 0.0);
  EXPECT_DOUBLE_EQ(command.vyaw, 0.0);
}

TEST(MotionConstraintsTest, OmnidirectionalModeRetainsConfiguredLateralVelocity)
{
  MotionConstraints constraints;
  constraints.motion_model = MotionModel::kOmnidirectional;
  constraints.forward_only = false;
  constraints.max_vx = 0.75;
  constraints.max_vy = 0.35;
  constraints.max_vyaw = 1.0;

  const PlanarCommand command = constrainPlanarCommand(-1.0, 0.6, 1.5, constraints);

  EXPECT_DOUBLE_EQ(command.vx, -0.75);
  EXPECT_DOUBLE_EQ(command.vy, 0.35);
  EXPECT_DOUBLE_EQ(command.vyaw, 1.0);
}

TEST(MotionConstraintsTest, InvalidMotionModelIsRejected)
{
  EXPECT_THROW(parseMotionModel("unknown"), std::invalid_argument);
}
}
