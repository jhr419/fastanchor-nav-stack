# TF Tree

Recommended FastAnchor TF tree:

```text
map
`-- camera_init
    `-- base_link
        `-- livox_frame
```

Responsibilities:

- FAST-LIO publishes `camera_init -> body`.
- `fast_anchor_localization_node` publishes `map -> camera_init` and
  `camera_init -> base_link` using the configured body/base calibration.
- `tf2_ros/static_transform_publisher` publishes `base_link -> livox_frame`.

The migrated node keeps compatibility parameters:

- `publish_tf`
- `publish_base_tf`
- `odom_coincident_with_base`

The bringup config sets `publish_base_tf: true` and
`odom_coincident_with_base: false` so the calibrated `base_link` frame is
available alongside FAST-LIO's `body` frame.
