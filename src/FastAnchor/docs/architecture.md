# Architecture

FastAnchor is organized as a ROS2 workspace with four in-house packages and two
third-party packages.

## In-house Packages

- `fast_anchor_localization`: migrated ICP localization node. It subscribes to
  FAST-LIO odometry and registered clouds, accepts an initial pose, runs the
  existing ICP flow, and publishes corrected localization outputs.
- `fast_anchor_bringup`: launch/config/RViz orchestration.
- `fast_anchor_interfaces`: custom status, ICP result, relocalization, and reset
  interfaces.
- `fast_anchor_tools`: standalone map and evaluation utility skeletons.

## Third-party Packages

- `src/third_party/FAST_LIO`
- `src/third_party/livox_ros_driver2`

These packages were moved only by path. Their internal code, package names,
CMake files, launch files, and configs are not modified by FastAnchor.

## Algorithm Migration Note

The previous `localization_adapter` node mixed ROS wrapping and ICP logic in one
source file. To avoid changing the working algorithm, the migrated
`fast_anchor_localization_node.cpp` intentionally keeps that flow together.
`fast_anchor_localization.cpp` is present as a future extraction point only.
