# Parameter Guide

Primary system config:

```text
src/fast_anchor_bringup/config/fast_anchor_localization.yaml
```

## Launch Arguments

Select the Livox model when starting the complete localization stack:

```bash
ros2 launch fast_anchor_bringup fast_anchor_mid360.launch.py lidar_model:=mid360s
```

`lidar_model` accepts `mid360` (default) and `mid360s`. It selects the matching
launch file from `livox_ros_driver2`; both models keep the existing FAST-LIO
Livox topic and message-type defaults.

## Localization Parameters

The node accepts both legacy flat parameters and grouped parameters. Important
legacy defaults from the original package were preserved:

- `scan_topic: /cloud_registered_body`
- `scan_leaf_size: 0.25`
- `map_leaf_size: 0.25`
- `min_scan_points: 120`
- `source_non_ground_filter_en: true`
- `source_non_ground_min_z: -0.2`
- `max_correspondence_distance: 2.0` in migrated YAML
- `transformation_epsilon: 0.01`
- `euclidean_fitness_epsilon: 0.01`
- `max_iterations: 40`
- `fitness_score_threshold: 2.0` in migrated YAML
- `relocalization_interval_s: 0.2`
- `output.aligned_cloud_interval_s: 0.0` in bringup YAML (publishes every input scan)
- `output.aligned_cloud_publish_rate_hz: 25.0` (`0.0` disables cached repeats)
- `output.path_publish_interval_s: 0.5` in bringup YAML (`0.0` publishes every pose)

The output intervals do not change ICP settings. With the default 0.2-second ICP
interval, ICP remains at no more than 5 Hz while intermediate input clouds are
transformed with the latest correction and published to SCAN at the sensor input
rate. A separate executor thread republishes the newest result at 25 Hz while
preserving its acquisition timestamp. Consequently, topic frequency can remain
above 20 Hz even when the sensor produces fewer unique scans.

Static map publishers use transient-local durability. Each unchanged map is
serialized once, when its first subscriber appears; it is not republished on a
periodic timer.

Map paths are intentionally relative examples. Override them at launch time:

```bash
ros2 launch fast_anchor_bringup localization_only.launch.py \
  map_pcd_path:=/absolute/path/map_preprocessed.pcd \
  visualization_map_pcd_path:=/absolute/path/map_visualization.pcd \
  icp_map_pcd_path:=/absolute/path/map_preprocessed.pcd
```
