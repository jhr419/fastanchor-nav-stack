#include <limits>

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

TEST(TwistCommandMapping, YawTakesPriorityAndLateralMotionStaysDisabled)
{
  CommandLimits limits;
  limits.forward_joystick_per_mps = 0.5;
  limits.yaw_joystick_per_rps = 0.2;
  limits.lateral_joystick_per_mps = std::numeric_limits<double>::quiet_NaN();
  limits.max_lateral_joystick = std::numeric_limits<double>::quiet_NaN();

  const auto command = convert_twist_to_longitudinal(0.6, -0.5, limits);

  EXPECT_FLOAT_EQ(command.forward, 0.0F);
  EXPECT_FLOAT_EQ(command.yaw, -0.10F);
  EXPECT_FLOAT_EQ(command.lateral, 0.0F);
}

TEST(TwistCommandMapping, AllowsForwardAndReverseAfterYawEntersDeadband)
{
  CommandLimits limits;
  limits.forward_joystick_per_mps = 0.5;
  limits.yaw_joystick_per_rps = 0.2;
  limits.angular_deadband = 0.05;

  const auto forward = convert_twist_to_longitudinal(0.6, 0.04, limits);
  const auto reverse = convert_twist_to_longitudinal(-0.6, -0.04, limits);

  EXPECT_FLOAT_EQ(forward.forward, 0.30F);
  EXPECT_FLOAT_EQ(forward.yaw, 0.0F);
  EXPECT_FLOAT_EQ(forward.lateral, 0.0F);
  EXPECT_FLOAT_EQ(reverse.forward, -0.30F);
  EXPECT_FLOAT_EQ(reverse.yaw, 0.0F);
  EXPECT_FLOAT_EQ(reverse.lateral, 0.0F);
}

TEST(TwistCommandMapping, ClampsOnlyEnabledAxes)
{
  CommandLimits limits;
  limits.limit_cmd_vel_input = true;
  limits.max_linear_x = 0.2;
  limits.max_angular_z = 0.4;
  limits.max_forward_joystick = 0.15;
  limits.max_yaw_joystick = 0.10;
  limits.exclusive_translation_rotation = false;

  const auto command = convert_twist_to_longitudinal(100.0, -100.0, limits);

  EXPECT_FLOAT_EQ(command.forward, 0.15F);
  EXPECT_FLOAT_EQ(command.yaw, -0.10F);
  EXPECT_FLOAT_EQ(command.lateral, 0.0F);
}

TEST(TwistCommandMapping, RejectsInvalidDeadband)
{
  CommandLimits limits;
  limits.angular_deadband = -0.01;

  EXPECT_THROW(convert_twist_to_longitudinal(0.0, 0.0, limits), std::invalid_argument);
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
