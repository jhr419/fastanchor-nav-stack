# 迁移代码审查

## 3dnav 全局规划功能确认

审查源位于 `3dnav_ws/src/global_planning`，确认三套正式后端：

- `nav3d_global_planning` A*：PCD/BT 载入，2.5D/3D 搜索，tomogram 可通行支持，
  TF/odom/topic 起点，三类目标输入，clearance/tomogram/smoothness cost，路径重建、
  去重、重采样、XYZ 平滑、平滑后碰撞验证与回退，以及 status/debug/marker。
- `pct_global_planner`：PCT tomogram 路由、原始/优化路径安全回退、clearance 与风险
  cost、自适应膨胀、路径平滑/重采样、风险 marker、debug artifacts 和重规划服务。
- `jie_3d_nav`：OctoMap A*、地面支撑、预阻塞代价、TF/手工起点、目标适配、
  clearance/risk marker 和统一 Path 适配。

这些源包复制到 FastPlanner 后没有修改原 `3dnav_ws`。唯一后端兼容扩展是 PCT
增加显式 portable/native 选择与 NumPy 中性地图缓存；原生 PCT 已实际完成路径
搜索、轨迹优化和统一可行性验证。

## MapProcessor planner test 确认

实现位置：

```text
src/map_processor_core/map_processor_core/planner_test_core.py
src/map_processor_core/map_processor_core/planner_test_node.py
src/map_processor_core/map_processor_core/planner_wrapper.py
src/map_processor_core/map_processor_core/pcd_io.py
```

数据结构为 `[>=5, slices, x, y]` 的 pickle，其中 `data[0]` 是 traversal cost，
`data[3]` 是 ground，`data[4]` 是 ceiling；索引由 center、resolution 和半尺寸偏移
换算。可移植 A* 在 8 邻域 XY 与相邻 layer 搜索，按 `max_ground_step` 限制跨层，
以 traversal cost 加权。

原输出包括 `planner_test_results.yaml`、Markdown summary、每条路径 CSV/path YAML、
ROS Path 和 Marker。FastPlanner 迁移核心搜索和输入解析，将运行态输出统一为
`feasibility_report.yaml` 与 `feasibility_points.csv`，并保留有限多 case 命令。

## 接口隔离

规划后端不再直接发布 `/planned_path`。launch 将其输出重定向到
`/fast_global_planner/raw_path`，统一节点在验证后发布三个兼容接口。这种结构允许
FastPlanner 独立运行，也允许 3dnav 局部规划器仍按原合同接入。
