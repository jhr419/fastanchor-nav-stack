# Navigation CPU performance

## What changed

The resource path targets repeated work rather than changing localization or
collision algorithms:

1. FastAnchor rejects input frames that are due for neither ICP nor aligned-cloud
   output before PointCloud2 conversion, filtering, and voxelization.
2. Static ICP and visualization maps are published once with transient-local QoS
   instead of being converted to PointCloud2 every second.
3. Localization path serialization is limited to 2 Hz by default.
4. SCAN occupancy visualization is limited to 5 Hz, builds both layers in one
   voxel traversal, and performs that traversal/serialization on a bounded
   background worker. The newest snapshot replaces an older pending snapshot.
5. The integrated launch does not start RViz unless explicitly requested.

Planning-critical rates remain unchanged: ICP 5 Hz, occupancy fusion 20 Hz,
collision checks 20 Hz, and closed-loop control 100 Hz.

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
- `/fast_anchor/aligned_cloud` remains near 5 Hz and grid fusion continues to
  accept updates without an increasing queue;
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
