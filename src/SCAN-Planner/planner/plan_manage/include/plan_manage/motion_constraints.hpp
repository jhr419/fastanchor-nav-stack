#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace scan_planner
{
enum class MotionModel
{
  kOmnidirectional,
  kNonholonomic
};

struct MotionConstraints
{
  MotionModel motion_model{MotionModel::kNonholonomic};
  bool forward_only{true};
  bool exclusive_translation_rotation{false};
  double max_vx{0.0};
  double max_vy{0.0};
  double max_vyaw{0.0};
  double linear_deadband{0.0};
  double angular_deadband{0.0};
};

struct PlanarCommand
{
  double vx{0.0};
  double vy{0.0};
  double vyaw{0.0};
};

inline MotionModel parseMotionModel(const std::string &value)
{
  if (value == "omnidirectional")
    return MotionModel::kOmnidirectional;
  if (value == "nonholonomic")
    return MotionModel::kNonholonomic;
  throw std::invalid_argument(
      "motion_model must be 'omnidirectional' or 'nonholonomic', got '" + value + "'");
}

inline const char *motionModelName(MotionModel model)
{
  return model == MotionModel::kNonholonomic ? "nonholonomic" : "omnidirectional";
}

inline PlanarCommand constrainPlanarCommand(
    double vx, double vy, double vyaw, const MotionConstraints &constraints)
{
  const double max_vx = std::max(0.0, constraints.max_vx);
  const double max_vy = std::max(0.0, constraints.max_vy);
  const double max_vyaw = std::max(0.0, constraints.max_vyaw);
  const double min_vx = constraints.forward_only ? 0.0 : -max_vx;

  PlanarCommand command;
  command.vx = std::clamp(vx, min_vx, max_vx);
  command.vy = constraints.motion_model == MotionModel::kNonholonomic
      ? 0.0 : std::clamp(vy, -max_vy, max_vy);
  command.vyaw = std::clamp(vyaw, -max_vyaw, max_vyaw);
  if (std::abs(command.vx) <= std::max(0.0, constraints.linear_deadband))
    command.vx = 0.0;
  if (std::abs(command.vy) <= std::max(0.0, constraints.linear_deadband))
    command.vy = 0.0;
  if (std::abs(command.vyaw) <= std::max(0.0, constraints.angular_deadband))
    command.vyaw = 0.0;
  if (constraints.exclusive_translation_rotation && command.vyaw != 0.0)
  {
    command.vx = 0.0;
    command.vy = 0.0;
  }
  return command;
}
}
