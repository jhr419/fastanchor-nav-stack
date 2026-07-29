# AGENT.md

This file is a compact project briefing for future coding agents working on
FastAnchor. Read it before scanning the whole workspace.

## Project Summary

FastAnchor is a ROS2 robot localization workspace. It combines:

- FAST-LIO frontend odometry and registered LiDAR clouds.
- ICP matching against an existing PCD map.
- A correction transform that anchors FAST-LIO odometry into the map frame.

The project was refactored from an older self-developed package named
`localization_adapter` under `src/localization`. The upstream packages were
moved under `src/third_party` without internal changes.

## Current Workspace Layout

```text
FastAnchor/
|-- README.md
|-- LICENSE
|-- AGENT.md
|-- docs/
|-- maps/
|-- bags/
|-- scripts/
`-- src/
    |-- fast_anchor_localization/
    |-- fast_anchor_bringup/
    |-- fast_anchor_interfaces/
    |-- fast_anchor_tools/
    `-- third_party/
        |-- FAST_LIO/
        `-- livox_ros_driver2/
```

## Packages

### `fast_anchor_localization`

Core localization package. Main executable:

```text
fast_anchor_localization_node
```

Important files:

- `src/fast_anchor_localization/src/fast_anchor_localization_node.cpp`
- `src/fast_anchor_localization/src/fast_anchor_localization.cpp`
- `src/fast_anchor_localization/config/fast_anchor_localization.yaml`
- `src/fast_anchor_localization/launch/localization_only.launch.py`

The original ICP algorithm was intentionally kept in
`fast_anchor_localization_node.cpp`. Do not casually split or rewrite it. The
placeholder `fast_anchor_localization.cpp` exists for a future careful
extraction, but the current behavior-protected implementation is still in the
node source.

### `fast_anchor_bringup`

System launch, runtime config, and RViz package.

Important files:

- `src/fast_anchor_bringup/launch/localization_only.launch.py`
- `src/fast_anchor_bringup/launch/fast_anchor_mid360.launch.py`
- `src/fast_anchor_bringup/launch/bag_localization.launch.py`
- `src/fast_anchor_bringup/launch/rviz.launch.py`
- `src/fast_anchor_bringup/config/fast_anchor_localization.yaml`
- `src/fast_anchor_bringup/rviz/fast_anchor.rviz`

Use this package for normal launching.

### `fast_anchor_interfaces`

Custom interfaces:

- `msg/LocalizationStatus.msg`
- `msg/IcpResult.msg`
- `srv/Relocalize.srv`
- `srv/ResetLocalization.srv`

`fast_anchor_localization` depends on this package.

### `fast_anchor_tools`

Standalone utility package. Current scripts are buildable placeholders for:

- PCD downsampling
- PCD cropping
- PCD transform
- trajectory evaluation
- ICP score plotting

Do not put localization runtime logic in this package.

### `src/third_party/FAST_LIO`

Upstream/third-party FAST-LIO package. ROS package name is `fast_lio`.

Do not modify internal code, launch, config, package.xml, or CMakeLists.txt
unless the user explicitly asks to patch upstream code.

### `src/third_party/livox_ros_driver2`

Upstream/third-party Livox ROS driver. ROS package name is
`livox_ros_driver2`.

Do not modify internal code, launch, config, package.xml, or CMakeLists.txt
unless the user explicitly asks to patch upstream code.

## Algorithm Protection Rules

The existing ICP + FAST-LIO localization behavior is considered good. Preserve:

- point cloud loading and map downsampling behavior
- source cloud filtering, range limits, height filter, and voxel leaf size
- FAST-LIO odometry as the ICP initial guess
- ICP parameters and acceptance checks
- `hasConverged()` and `fitness_score_threshold_` rejection logic
- `map_to_odom_ = refined_map_to_base * latest_odom_to_base_.inverse()`
- initial pose handling
- reset behavior and FAST-LIO reset client
- debug logs and point cloud publishers

If a requested refactor risks changing the algorithm, prefer a minimal package
or build-system change and document the deferred extraction.

## Runtime Data Flow

Default inputs:

- `/Odometry`: FAST-LIO odometry.
- `/cloud_registered_body`: FAST-LIO registered cloud in body frame.
- `/initialpose`: initial map-frame pose from RViz/Nav2.

Bringup default outputs:

- `/fast_anchor/pose`
- `/fast_anchor/odom`
- `/fast_anchor/path`
- `/fast_anchor/status`
- `/fast_anchor/icp_result`
- `/fast_anchor/aligned_cloud`
- `/fast_anchor/local_map`
- `/fast_anchor/global_map`

The node still accepts legacy flat parameters and legacy output topic defaults
when launched without the bringup YAML.

## TF Ownership

Recommended tree:

```text
map -> odom -> base_link -> livox_frame
```

Recommended ownership:

- FAST-LIO publishes `odom -> base_link`.
- FastAnchor publishes `map -> odom`.
- Static transform publisher publishes `base_link -> livox_frame`.

Compatibility parameters exist:

- `publish_tf`
- `publish_base_tf`
- `odom_coincident_with_base`

The bringup config sets `publish_base_tf: false` and
`odom_coincident_with_base: false`. The localization package config preserves
the old adapter defaults for easier behavior comparison.

## Important Parameters

Original behavior-critical values migrated from the old config:

```yaml
scan_topic: "/cloud_registered_body"
base_to_body_xyz: [-0.3, 0.0, -0.2]
base_to_body_rpy: [0.0, 0.0, 0.0]
min_range: 0.5
max_range: 60.0
scan_leaf_size: 0.25
map_leaf_size: 0.25
min_scan_points: 120
source_non_ground_filter_en: true
source_non_ground_min_z: -0.2
max_correspondence_distance: 2.0
transformation_epsilon: 0.01
euclidean_fitness_epsilon: 0.01
max_iterations: 40
fitness_score_threshold: 2.0
relocalization_interval_s: 0.2
```

Runtime output throttles used by the integrated performance profile:

```yaml
output.aligned_cloud_interval_s: 0.0  # every fresh input scan; ICP stays rate-limited
output.aligned_cloud_publish_rate_hz: 25.0  # fixed cached output; 0 disables repeats
output.path_publish_interval_s: 0.5   # 0 restores publication on every pose
```

Fixed-rate repeats preserve the original acquisition timestamp. SCAN uses this
stamp to prevent the same observation from changing occupancy log odds more than
once. The fixed-rate timer runs in a separate callback group on the node's
two-thread executor so ICP cannot block the publication cadence.

Static map clouds use transient-local QoS and are serialized once, after a
subscriber appears. Do not restore periodic full-map serialization unless the
map becomes mutable.

Primary runtime YAML:

```text
src/fast_anchor_bringup/config/fast_anchor_localization.yaml
```

## Build And Run

Build:

```bash
colcon build --symlink-install
source install/setup.bash
```

Localization only:

```bash
ros2 launch fast_anchor_bringup localization_only.launch.py
```

MID360 system:

```bash
ros2 launch fast_anchor_bringup fast_anchor_mid360.launch.py
```

Bag test:

```bash
ros2 launch fast_anchor_bringup bag_localization.launch.py bag_path:=/path/to/bag
```

## Known Migration Notes

- `fast_anchor_localization_node.cpp` still holds the core implementation by
  design. This is not accidental.
- The node supports both old flat parameters and newer grouped parameters such
  as `topics.fast_lio_odom`.
- Third-party launch inclusion assumes:
  - `livox_ros_driver2/launch_ROS2/msg_MID360_launch.py`
  - `fast_lio/launch/mapping.launch.py`
- If those upstream launch files differ in another checkout, update bringup or
  pass custom launch composition. Do not edit third-party internals just for
  FastAnchor launch convenience.
