# Topic Interface

## Inputs

- FAST-LIO odometry: `/Odometry`
- FAST-LIO registered cloud: `/cloud_registered_body`
- Initial pose: `/initialpose`

These are configured through `topics.fast_lio_odom`,
`topics.fast_lio_cloud`, and `topics.initial_pose`.

## Outputs

Bringup defaults:

- `/fast_anchor/pose`: corrected pose.
- `/fast_anchor/odom`: corrected odometry-style pose.
- `/fast_anchor/path`: accumulated corrected path.
- `/fast_anchor/status`: `fast_anchor_interfaces/msg/LocalizationStatus`.
- `/fast_anchor/icp_result`: `fast_anchor_interfaces/msg/IcpResult`.
- `/fast_anchor/aligned_cloud`: latest ICP aligned cloud.
- `/fast_anchor/local_map`: ICP target map debug cloud.
- `/fast_anchor/global_map`: visualization map debug cloud.

The node still accepts legacy flat output parameters such as `pose_topic` and
`aligned_cloud_topic`.
