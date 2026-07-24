# fast_anchor_localization

Core FastAnchor localization package.

The executable `fast_anchor_localization_node` is the migrated form of the
previous `localization_adapter/icp_localization_node`. The original ICP
registration flow is intentionally kept inside
`src/fast_anchor_localization_node.cpp` during this refactor to avoid changing
the proven localization behavior.

## Inputs

- FAST-LIO odometry: `/Odometry`
- FAST-LIO registered body cloud: `/cloud_registered_body`
- Initial pose: `/initialpose`

All topic names are configurable through YAML. The node accepts both the old
flat parameter names and the newer grouped names such as `topics.fast_lio_odom`
and `topics.output_pose`.

## Outputs

- Pose, odometry, and path
- `fast_anchor_interfaces/msg/LocalizationStatus`
- `fast_anchor_interfaces/msg/IcpResult`
- Aligned cloud and map debug clouds
- TF correction according to `publish_tf`, `publish_base_tf`, and
  `odom_coincident_with_base`

## Launch

```bash
ros2 launch fast_anchor_localization localization_only.launch.py
```

For full system launch, prefer `fast_anchor_bringup`.
