# Navigation CPU performance

## What changed

The resource path targets repeated work rather than changing localization or
collision algorithms:

1. FastAnchor keeps ICP rate-limited, processes every fresh LiDAR input, and
   publishes the latest aligned cloud at a fixed 25 Hz on a separate executor
   thread. Repeated messages preserve the acquisition timestamp.
2. Static ICP and visualization maps are published once with transient-local QoS
   instead of being converted to PointCloud2 every second.
3. Localization path serialization is limited to 2 Hz by default.
4. SCAN occupancy visualization is limited to 5 Hz, builds both layers in one
   voxel traversal, and performs that traversal/serialization on a bounded
   background worker. The newest snapshot replaces an older pending snapshot.
5. The integrated launch does not start RViz unless explicitly requested.

Planning-critical rates remain unchanged: ICP 5 Hz, occupancy fusion 20 Hz,
collision checks 20 Hz, and closed-loop control 100 Hz.

The fixed 25 Hz cloud output increases DDS serialization and transport work even
when the sensor frame is unchanged. SCAN rejects repeated acquisition stamps, so
this output requirement does not multiply raycasting or occupancy evidence, but
other subscribers should also avoid treating every publication as a fresh scan.

## Workstation evidence

All changed packages were built in Release mode and the SCAN launch tests passed.
An isolated SCAN visualization check used a 0.10 m, 10 x 10 x 5 m grid with one
occupancy subscriber and no sensor input. At 20 Hz the planner main thread and
visualization worker consumed approximately 9% of one workstation CPU in total;
at 5 Hz they consumed approximately 4%, a reduction of about 57% for this isolated
workload. The worker and main thread were observed on different logical CPUs.

This is not evidence that the complete system has already reached a 50% reduction
on the aircraft computer. ICP, FAST-LIO, the Livox driver, memory bandwidth, CPU
frequency, and live point density are absent from the isolated check.

## Onboard acceptance test

Use the same map, route, LiDAR rate, initial pose, goal, power mode, and test
duration for both branches. Disable unrelated applications. After a 30-second
warm-up, collect at least 60 seconds:

```bash
bash scripts/profile_navigation_cpu.sh 60 | tee navigation_cpu.txt
```

Also record per-core utilization with the platform's normal monitor (`top -H`,
`htop`, or `pidstat -t` when available), topic rates, localization fitness, local
planning success, and control rate. Compute reduction from the sum of navigation
process CPU percentages:

```text
reduction = (baseline_cpu - optimized_cpu) / baseline_cpu * 100%
```

Acceptance requires all of the following:

- aggregate navigation CPU reduction is at least 50%;
- no sustained 100% utilization on a single logical CPU;
- `/fast_anchor/aligned_cloud` remains above 20 Hz after the first valid cloud;
- SCAN integrates each unique acquisition timestamp once rather than treating
  cached 25 Hz repeats as independent sensor evidence;
- ICP fitness/rejection behavior and path success are not materially worse;
- `/cmd_vel` remains near 100 Hz while executing a trajectory.

The profiling script prints each process's inherited `Cpus_allowed_list`. If it
contains only one CPU, the launch shell or service is pinning every child process
to that CPU; application threads cannot escape that mask. Fix the service/cgroup
configuration, or launch with an explicit valid range for that platform, for
example:

```bash
taskset --cpu-list 0-3 ros2 launch navigation_bringup navigation_system.launch.py \
  map_pcd_path:=$PWD/maps/map_preprocessed2.pcd
```

Do not copy `0-3` blindly: use online CPU IDs shown by `lscpu -e` and leave room
for flight-control or real-time workloads required by the platform.

If the platform still saturates, first run headless with
`grid_visualization_rate_hz:=0`. Only after measuring that result should ICP rate,
map resolution, or point-cloud leaf sizes be changed, because those knobs alter
localization or collision behavior.

## Runtime map diagnostics

The integrated launch now enables a five-second runtime diagnostic window by
default. It measures the unique local obstacle-map fusion rate separately from
the fixed-rate aligned-cloud transport and from the optional occupancy
visualization. This matters because FastAnchor preserves the acquisition stamp
when it republishes a cached cloud; those repeats are transport frames, not new
map observations.

Each window writes two ROS log records:

- `[RuntimeLog]` reports `OK` or `DEGRADED`, input/prepared/fusion rates,
  coalesced frames, process and whole-system CPU, memory, RSS, load average,
  Linux CPU/memory pressure, visualization rate, and an automatic `reason`.
- `[RuntimeLog][Stages]` reports average/maximum time for point-cloud decoding,
  filtering, sliding-map updates, depth projection, raycasting/inflation, fusion
  queue delay, and occupancy visualization serialization.

The default fusion target is 10 Hz with a 90% acceptance tolerance, matching a
typical MID360 unique acquisition rate. Change it to the actual sensor contract,
not the cached publication rate:

```bash
ros2 launch navigation_bringup navigation_system.launch.py \
  map_pcd_path:=$PWD/maps/map_preprocessed2.pcd \
  runtime_log_map_target_rate_hz:=10.0 \
  runtime_log_report_interval_sec:=5.0 \
  runtime_log_csv_path:=/tmp/navigation-runtime.csv
```

`runtime_log_csv_path` is empty by default; ROS logs are still persisted by the
normal ROS 2 logging system. The CSV is append-only and contains one row per
window for plotting or comparing runs. Set `runtime_log_enabled:=false` to
disable both resource sampling and stage reports.

Common reason values are:

| Reason | Interpretation |
|---|---|
| `upstream_unique_input_low` | The planner received fewer genuinely new sensor frames than required. |
| `upstream_repeated_cloud` | A significant part of the input was cached output with a repeated acquisition stamp. |
| `missing_sensor_pose` | Clouds arrived before a valid pose, so they could not be fused. |
| `empty_or_fully_filtered_input` | The cloud was empty or no point survived local range/finite-value filtering. |
| `fusion_queue_coalescing` | New prepared frames replaced pending data before the 20 Hz fusion callback ran. |
| `slow_pointcloud_*` / `slow_sliding_map` | Input conversion, filtering, or map shifting consumed the frame budget. |
| `slow_raycast_and_inflation` | Ray traversal and occupancy/inflation updates dominated the frame budget. |
| `executor_callback_delay` | The single-threaded planner executor delayed fusion even though map work was ready. |
| `visualization_worker_backlog` / `visualization_serialization` | The bounded visualization worker dropped old snapshots, or snapshot/PointCloud2 conversion consumed too much of its period. |
| `visualization_timer_or_executor_delay` | Visualization work itself was short, but the single-threaded timer was invoked too slowly. |
| `system_cpu_saturation` / `cpu_scheduling_pressure` | Other work on the machine competed for CPU time. |
| `planner_process_cpu_core_saturation` | `scan_planner_node` itself used at least about one full CPU core while the rate was low. |
| `memory_pressure` | Available memory or Linux memory-stall pressure crossed the warning threshold. |

Visualization is shown as `inactive` when neither occupancy topic has a
subscriber. That is not a map-rate failure: collision queries use the internal
fusion buffer and do not require RViz point-cloud publication.
