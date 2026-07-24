# fast_anchor_interfaces

Custom ROS2 interfaces used by FastAnchor.

- `LocalizationStatus.msg`: localization state, ICP convergence, and debug metrics.
- `IcpResult.msg`: latest ICP acceptance result, corrected pose, and map-to-odom transform.
- `Relocalize.srv`: applies an externally supplied initial pose.
- `ResetLocalization.srv`: resets localization state through a typed FastAnchor service.
