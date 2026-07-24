# FastPlanner 可行性检测

## 1. 原理与 MapProcessor 的关系

FastPlanner 原样迁移了 MapProcessor planner test 的三个可复用核心：

- `TomogramMap`：读取并验证 PCT pickle 的 `data/resolution/center/slice_h0/slice_dh`；
- `TomogramAStarPlanner`：在多层 tomogram 中按 ground step 和 traversability cost 搜索；
- PCD ASCII、binary、binary-compressed XYZ reader。

MapProcessor 的判定定义保持不变：有限 ground 且 traversal cost 不高于阈值的
单元为可通行，有限 ground 但高代价为占用，其余为 unknown。

FastPlanner 在此基础上增加路径级验证：

1. 按 `path_check_resolution` 对每个路径段插值，避免只检查稀疏 waypoint；
2. 检查 PCD XYZ 边界和 tomogram XY 边界；
3. 选择与真实 z 最近的 tomogram layer，检查 traversal cost 和 ground z 误差；
4. 只把路径点以上 `obstacle_min_relative_z .. obstacle_max_relative_z` 的 PCD 点
   当作机身障碍，避免地面点被误报为碰撞；
5. 计算 XY clearance，小于 `feasibility_clearance_threshold` 即碰撞；
6. 检查相邻原始 waypoint 是否超过 `max_path_segment_length`，标记断裂段。

OctoMap `.bt` 的存在会写入报告，实际逐点验证以 tomogram 的可通行语义和 PCD
的真实障碍距离为准；这与 MapProcessor 当前 `.bt` 仅保存占用端点的约束一致。

可选原生 PCT 因 pybind 依赖 NumPy 1.x，launch 会先把同一个 MapProcessor pickle
转换为项目内中性 NPZ 缓存，再让 NumPy 1.x 进程读取；原地图文件保持不变。

## 2. 使用模式

配置文件：`src/fast_global_planner/config/global_planner.yaml`。

```yaml
enable_feasibility_check: true
feasibility_check_mode: "post"   # pre / post / both
feasibility_clearance_threshold: 0.2
fail_on_infeasible_path: true
publish_feasibility_debug: true
```

- `pre`：验证当前起点和目标点，通过后才将目标转发给规划后端；后端路径直接发布。
- `post`：目标直接转发；后端路径通过完整逐段验证后才发布。
- `both`：同时执行上述两个阶段。

当 `fail_on_infeasible_path=true` 时，不可行路径不会出现在稳定输出上，同时发布
空 `/planned_path` 和 `/path` 清除旧路径。设为 `false` 时仍会报告失败和红色点，
但允许路径发布，适合调参，不适合真实执行。

## 3. 状态和可视化

```bash
ros2 topic echo /fast_global_planner/feasibility_status
```

典型结果：

```text
PATH_FEASIBLE feasible=true min_clearance=0.432 collision_points=0 ...
PATH_INFEASIBLE feasible=false min_clearance=0.124 collision_points=8 ...
```

RViz 配置已加入 `/fast_global_planner/feasibility_marker`：绿色表示通过，红色表示
clearance 碰撞，橙色表示边界、tomogram 或 z/连通性失败。

## 4. 文件调试

每次 pre/post 检查更新：

```text
debug/feasibility/feasibility_report.yaml
debug/feasibility/feasibility_points.csv
```

YAML 记录地图元数据、总点数、插值点数、最小 clearance、碰撞数、断裂数和全部
失败原因。CSV 每行记录 XYZ、tomogram index 对应状态、cost、ground z、clearance
和原因，可直接用于离线绘图或 AI 分析。

有限离线测试：

```bash
bash scripts/run_feasibility_test.sh --clearance-threshold 0.2
```

该命令复用 MapProcessor 的 `planner_test_cases.yaml`，先执行迁移的多层 A*，再对
每条结果执行 FastPlanner 后验证。只要任一规划或验证失败，命令返回非零，同时
仍会完整写出 YAML/CSV。

MapProcessor 原测试用例只按 tomogram traversability 选点，并未按 `0.2 m`
clearance 设计；因此迁移后的同一用例可能“规划成功但可行性失败”。这正是新增
安全闸门应暴露的结果，不应通过降低阈值或吞掉失败来伪造成功。

## 5. 调参顺序

先查看 `traversal_cost` 和 `ground_z`，确认路径落在正确楼层；再查看 clearance。
若地面被误报，检查相对高度范围，而不是直接关闭碰撞检测。若窄通道全部失败，
根据机器人真实外形调整 clearance threshold，并保留 `fail_on_infeasible_path=true`
做最终验收。
