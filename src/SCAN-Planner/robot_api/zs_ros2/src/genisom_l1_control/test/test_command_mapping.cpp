#include <limits>

#include "genisom_l1_control/highlevel_velocity_client.hpp"
#include "genisom_l1_control/sdk_wrapper.hpp"
#include "gtest/gtest.h"

namespace genisom_l1_control
{

TEST(CommandMapping, MapsRosAxesToOfficialJoystickOrder)
{
  CommandLimits limits;
  const auto command = convert_twist_to_normalized(0.05, -0.025, 0.10, limits);

  EXPECT_FLOAT_EQ(command.forward, 0.05F);
  EXPECT_FLOAT_EQ(command.yaw, 0.05F);
  EXPECT_FLOAT_EQ(command.lateral, -0.05F);
}

TEST(CommandMapping, ClampsPhysicalAndJoystickLimits)
{
  CommandLimits limits;
  limits.limit_cmd_vel_input = true;
  limits.max_forward_joystick = 0.10;
  limits.max_lateral_joystick = 0.10;
  limits.max_yaw_joystick = 0.10;
  const auto command = convert_twist_to_normalized(100.0, -100.0, 100.0, limits);

  EXPECT_FLOAT_EQ(command.forward, 0.10F);
  EXPECT_FLOAT_EQ(command.yaw, 0.10F);
  EXPECT_FLOAT_EQ(command.lateral, -0.10F);
}

TEST(CommandMapping, ConvertsNonFiniteAxisToZero)
{
  CommandLimits limits;
  const auto command = convert_twist_to_normalized(
    std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0, limits);

  EXPECT_FLOAT_EQ(command.forward, 0.0F);
}

TEST(CommandMapping, RejectsInvalidLimits)
{
  CommandLimits limits;
  limits.limit_cmd_vel_input = true;
  limits.max_linear_x = 0.0;
  EXPECT_THROW(convert_twist_to_normalized(0.0, 0.0, 0.0, limits), std::invalid_argument);
}

TEST(CommandMapping, DisabledInputLimitUsesIndependentCalibrationGains)
{
  CommandLimits limits;
  limits.limit_cmd_vel_input = false;
  limits.forward_joystick_per_mps = 0.5;
  limits.lateral_joystick_per_mps = 0.25;
  limits.yaw_joystick_per_rps = 0.2;

  const auto command = convert_twist_to_normalized(0.6, -0.4, 0.5, limits);

  EXPECT_FLOAT_EQ(command.forward, 0.30F);
  EXPECT_FLOAT_EQ(command.yaw, 0.10F);
  EXPECT_FLOAT_EQ(command.lateral, -0.10F);
}

TEST(CommandMapping, AlwaysClampsToOfficialJoystickProtocolRange)
{
  CommandLimits limits;
  limits.limit_cmd_vel_input = false;
  const auto command = convert_twist_to_normalized(100.0, -100.0, 100.0, limits);

  EXPECT_FLOAT_EQ(command.forward, 1.0F);
  EXPECT_FLOAT_EQ(command.yaw, 1.0F);
  EXPECT_FLOAT_EQ(command.lateral, -1.0F);
}

TEST(CommandMapping, RejectsInvalidCalibrationGain)
{
  CommandLimits limits;
  limits.forward_joystick_per_mps = 0.0;
  EXPECT_THROW(convert_twist_to_normalized(0.0, 0.0, 0.0, limits), std::invalid_argument);
}

TEST(TwistCommandMapping, PassesPhysicalVelocityWithoutYawPriority)
{
  VelocityLimits limits;

  const auto command = convert_twist_to_velocity(0.6, -0.2, -0.5, limits);

  EXPECT_FLOAT_EQ(command.linear_x, 0.6F);
  EXPECT_FLOAT_EQ(command.linear_y, -0.2F);
  EXPECT_FLOAT_EQ(command.angular_z, -0.5F);
}

TEST(TwistCommandMapping, AppliesDeadbandsPerAxisWithoutMutualExclusion)
{
  VelocityLimits limits;
  limits.linear_deadband = 0.05;
  limits.angular_deadband = 0.05;

  const auto command = convert_twist_to_velocity(0.6, 0.04, -0.5, limits);

  EXPECT_FLOAT_EQ(command.linear_x, 0.6F);
  EXPECT_FLOAT_EQ(command.linear_y, 0.0F);
  EXPECT_FLOAT_EQ(command.angular_z, -0.5F);
}

TEST(TwistCommandMapping, ClampsPhysicalVelocityLimits)
{
  VelocityLimits limits;
  limits.limit_cmd_vel_input = true;
  limits.max_linear_x = 0.2;
  limits.max_linear_y = 0.3;
  limits.max_angular_z = 0.4;

  const auto command = convert_twist_to_velocity(100.0, -100.0, -100.0, limits);

  EXPECT_FLOAT_EQ(command.linear_x, 0.2F);
  EXPECT_FLOAT_EQ(command.linear_y, -0.3F);
  EXPECT_FLOAT_EQ(command.angular_z, -0.4F);
}

TEST(TwistCommandMapping, RejectsInvalidDeadband)
{
  VelocityLimits limits;
  limits.angular_deadband = -0.01;

  EXPECT_THROW(convert_twist_to_velocity(0.0, 0.0, 0.0, limits), std::invalid_argument);
}

TEST(ModelPolicy, WheelModelsRejectOfficiallyUnsupportedActions)
{
  for (const auto model :
    {zsibot::Model::MODEL_XGW, zsibot::Model::MODEL_XGWHSPD})
  {
    EXPECT_FALSE(command_supported_for_model(zsibot::CmdCode::CMD_JUMP, model));
    EXPECT_FALSE(command_supported_for_model(zsibot::CmdCode::CMD_FORWARD_JUMP, model));
    EXPECT_FALSE(command_supported_for_model(zsibot::CmdCode::CMD_BACK_FLIP, model));
    EXPECT_FALSE(command_supported_for_model(zsibot::CmdCode::CMD_GREET, model));
    EXPECT_FALSE(command_supported_for_model(zsibot::CmdCode::CMD_TWO_LEG_STAND, model));
    EXPECT_TRUE(command_supported_for_model(zsibot::CmdCode::CMD_CRAWL_FORWARD, model));
    EXPECT_TRUE(
      command_supported_for_model(
        zsibot::CmdCode::CMD_CLIMBING_HIGH_PLATFORM, model));
    EXPECT_TRUE(command_supported_for_model(zsibot::CmdCode::CMD_UNLOAD_SQUAT, model));
  }
}

TEST(ModelPolicy, PointFootRejectsOfficiallyUnsupportedActions)
{
  const auto model = zsibot::Model::MODEL_XG;
  EXPECT_FALSE(command_supported_for_model(zsibot::CmdCode::CMD_CRAWL_FORWARD, model));
  EXPECT_FALSE(
    command_supported_for_model(
      zsibot::CmdCode::CMD_CLIMBING_HIGH_PLATFORM, model));
  EXPECT_FALSE(command_supported_for_model(zsibot::CmdCode::CMD_HAND_STAND, model));
  EXPECT_TRUE(command_supported_for_model(zsibot::CmdCode::CMD_JUMP, model));
}

TEST(ModelPolicy, LabModeRequirementMatchesOfficialDocumentation)
{
  EXPECT_TRUE(command_requires_lab_mode(zsibot::CmdCode::CMD_JUMP));
  EXPECT_TRUE(command_requires_lab_mode(zsibot::CmdCode::CMD_UNLOAD_SQUAT));
  EXPECT_FALSE(command_requires_lab_mode(zsibot::CmdCode::CMD_CRAWL_FORWARD));
  EXPECT_FALSE(command_requires_lab_mode(zsibot::CmdCode::CMD_CLIMBING_HIGH_PLATFORM));
}

TEST(VelocityBridgePolicy, OnlySdkOwnerWithAllSafetyGatesMaySend)
{
  EXPECT_TRUE(
    velocity_bridge_may_send(
      true, true, false, false, zsibot::FunctionMode::FM_SDK));
  EXPECT_FALSE(
    velocity_bridge_may_send(
      true, true, false, false, zsibot::FunctionMode::FM_REMOTE));
  EXPECT_FALSE(
    velocity_bridge_may_send(
      true, true, false, false, zsibot::FunctionMode::FM_GENERAL_SDK));
  EXPECT_FALSE(
    velocity_bridge_may_send(
      true, true, false, false, zsibot::FunctionMode::FM_ROAMER));
  EXPECT_FALSE(
    velocity_bridge_may_send(
      false, true, false, false, zsibot::FunctionMode::FM_SDK));
  EXPECT_FALSE(
    velocity_bridge_may_send(
      true, false, false, false, zsibot::FunctionMode::FM_SDK));
  EXPECT_FALSE(
    velocity_bridge_may_send(
      true, true, true, false, zsibot::FunctionMode::FM_SDK));
  EXPECT_FALSE(
    velocity_bridge_may_send(
      true, true, false, true, zsibot::FunctionMode::FM_SDK));
}

TEST(TwistTakeoverPolicy, ExitsAfterConfirmedSdkOwnerChanges)
{
  EXPECT_FALSE(sdk_control_lost(false, zsibot::FunctionMode::FM_REMOTE));
  EXPECT_FALSE(sdk_control_lost(true, zsibot::FunctionMode::FM_SDK));
  EXPECT_TRUE(sdk_control_lost(true, zsibot::FunctionMode::FM_REMOTE));
  EXPECT_TRUE(sdk_control_lost(true, zsibot::FunctionMode::FM_GENERAL_SDK));
  EXPECT_TRUE(sdk_control_lost(true, zsibot::FunctionMode::FM_NULL));
}

}  // 命名空间 genisom_l1_control
