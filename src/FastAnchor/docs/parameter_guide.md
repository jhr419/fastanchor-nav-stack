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

Map paths are intentionally relative examples. Override them at launch time:

```bash
ros2 launch fast_anchor_bringup localization_only.launch.py \
  map_pcd_path:=/absolute/path/map_preprocessed.pcd \
  visualization_map_pcd_path:=/absolute/path/map_visualization.pcd \
  icp_map_pcd_path:=/absolute/path/map_preprocessed.pcd
```
