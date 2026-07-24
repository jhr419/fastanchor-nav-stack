# AGENT.md

Read this file before changing FastPlanner. Update it after any task that changes
behavior, interfaces, launch flow, parameters, map contracts, outputs, dependencies,
or debugging workflow.

## Purpose and invariants

FastPlanner is a standalone ROS 2 Humble global-planning workspace extracted from
`3dnav_ws`. It must build without sourcing a navigation workspace. Never add a
runtime dependency on the original workspace and never hard-code a user home path.
The original workspace is an upstream reference only and must not be modified.

Stable outputs are:

```text
/planned_path         nav_msgs/msg/Path
/path                 nav_msgs/msg/Path
/planned_path_marker  visualization_msgs/msg/Marker
```

All output paths use `frame_id: map`, retain real XYZ, and are ordered start to goal.
Only `fast_global_planner_node` publishes the stable outputs. Backends publish to
`/fast_global_planner/raw_path`, which is gated by feasibility validation.

## Structure

```text
src/fast_planner_common       migrated MapProcessor tomogram A*, PCD IO, checker
src/fast_global_planner      goal gate, path gate, launch, config, RViz
src/3dnav_global_planning    extracted 3dnav A* backend (ROS package nav3d_global_planning)
src/pct_global_planner       extracted PCT adapter and path postprocessor
src/pct_planner_ros2         PCT wrapper required by the optional native backend
src/jie_3d_nav               extracted jie OctoMap packages
vendor/pct_planner           project-local optional native PCT runtime
docs                         architecture and feasibility documentation
debug/feasibility            current YAML/CSV validation evidence
```

## MapProcessor contract

Default inputs are resolved against `MAP_PROCESSOR_ROOT`, or against the sibling
`../MapProcessor` project:

```text
maps/output/map_preprocessed.pcd
maps/output/map_preprocessed.bt
maps/tomogram/map_preprocessed.pickle
```

The PCD and pickle are required. The `.bt` is required only for the preferred
`jie_octomap` flow; A* falls back to PCD when it is absent.

## Backends

- `astar` (default): extracted 3dnav 2.5D/3D A*, PCD/BT occupancy, tomogram support,
  clearance/risk costs, resampling, smoothing, collision validation, debug markers.
- `pct`: extracted PCT ROS adapter, native planner, optimizer, and postprocessor.
  Native PCT is the default for this selection. Launch converts the MapProcessor
  pickle into a project-local neutral NPZ cache under `debug/native_pct/` before
  starting the NumPy-1 native process. `use_native_pct_backend:=false` selects the
  portable multi-layer tomogram A* fallback.
- `jie_octomap`: extracted jie OctoMap planner, adapter, map conversion, clearance,
  and risk/debug outputs.

## Feasibility behavior

Configuration is `src/fast_global_planner/config/global_planner.yaml`.

```yaml
enable_feasibility_check: true
feasibility_check_mode: "post"  # pre / post / both
feasibility_clearance_threshold: 0.2
fail_on_infeasible_path: true
publish_feasibility_debug: true
```

Pre mode filters the three public goal inputs before forwarding them to a backend.
Post mode validates interpolated path samples before publishing stable outputs.
Both mode does both. A rejected path publishes an empty Path to clear stale data.

Debug interfaces:

```text
/fast_global_planner/feasibility_status  std_msgs/msg/String
/fast_global_planner/feasibility_marker  visualization_msgs/msg/Marker
debug/feasibility/feasibility_report.yaml
debug/feasibility/feasibility_points.csv
```

## Commands

The helper scripts temporarily disable Bash `nounset` while sourcing ROS 2 Humble
and workspace setup files because those upstream scripts may read unset variables.

```bash
bash scripts/build.sh
source install/setup.bash
ros2 launch fast_global_planner global_planner.launch.py algorithm:=astar
ros2 launch fast_global_planner global_planner.launch.py algorithm:=pct
ros2 launch fast_global_planner global_planner.launch.py algorithm:=jie_octomap
bash scripts/run_feasibility_test.sh --clearance-threshold 0.2
```

## Verification rules

- Run Python syntax checks and `colcon build --symlink-install` after changes.
- Run the finite offline feasibility test and at least an A* launch smoke test.
- Check `/planned_path`, `/path`, `/planned_path_marker`, feasibility status, and
  feasibility marker types.
- Do not commit build/install/log, generated reports, core dumps, or copied maps.
- Preserve the backend source behavior unless a compatibility adapter is documented.

## Verified baseline

On 2026-07-23 all eight packages built independently. A* produced an 11-point,
1.960 m path with 0.813 m checked minimum clearance. Native PCT loaded the neutral
NPZ cache, ran A*/trajectory optimization, and produced an 8-point, 1.963 m path
that passed the common gate with 0.806 m minimum clearance. jie loaded the
MapProcessor `.bt` with 66,057 occupied leaves. Pre and post rejection paths,
empty-path clearing, topic types, YAML/CSV output, and three synthetic tests were
also verified.
