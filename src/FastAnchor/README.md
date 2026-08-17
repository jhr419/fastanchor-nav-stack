# FastAnchor

FastAnchor 是一个基于 FAST-LIO 前端里程计与 ICP 地图匹配锚定的 ROS2 机器人定位系统。

项目的核心思想是：由 FAST-LIO 提供连续、实时的 LiDAR-Inertial Odometry，再通过 ICP 将当前点云与已有 PCD 地图匹配，计算并发布 `map -> odom` 修正关系，从而把局部里程计稳定锚定到全局地图坐标系中。

## 功能包结构

- `fast_anchor_localization`：核心 ICP 定位逻辑包。
- `fast_anchor_bringup`：系统启动、参数配置和 RViz 入口包。
- `fast_anchor_interfaces`：自定义消息与服务接口包。
- `fast_anchor_tools`：地图处理、bag 测试、轨迹评估等工具包。
- `yifan_lio_adapter`：把 yifanLIO 输出适配为 FastAnchor 统一 LIO 接口的轻量 adapter。
- `src/third_party/FAST_LIO`：第三方 FAST-LIO 前端里程计包。
- `src/third_party/lio`：第三方 yifanLIO 前端里程计包（ESKF + ikd-Tree）。
- `src/third_party/livox_ros_driver2`：第三方 Livox ROS2 雷达驱动包。

第三方包保持上游结构不变（yifanLIO 仅新增 `publish_tf` 接口开关）。FastAnchor 只通过 ROS2 topic、参数、launch 文件和 TF 与它们交互。

## 算法保护说明

原有 ICP + FAST-LIO 定位算法表现已经较好。本次重构只整理工程结构、包组织、构建系统、启动文件和文档，不应改变核心定位行为。

请不要随意修改以下内容：

- ICP 匹配流程。
- 点云预处理、滤波、降采样、距离裁剪和高度裁剪策略。
- FAST-LIO 里程计作为 ICP 初值的使用方式。
- ICP fitness score 判断逻辑和阈值。
- `map -> odom`、`odom -> base_link` 相关位姿计算逻辑。
- 现有定位结果、调试点云、状态和日志发布语义。

当前核心实现仍保留在：

```text
src/fast_anchor_localization/src/fast_anchor_localization_node.cpp
```

`fast_anchor_localization.cpp` 目前仅作为后续谨慎拆分核心算法的预留文件。

## 构建

在工作空间根目录执行：

```bash
colcon build --symlink-install
source install/setup.bash
```

## 运行

如果 Livox 驱动和 FAST-LIO 已经单独启动，只启动 FastAnchor 定位节点：

```bash
ros2 launch fast_anchor_bringup localization_only.launch.py
```

Livox 真机一键启动（默认 `mid360`）：

```bash
ros2 launch fast_anchor_bringup fast_anchor_mid360.launch.py
```

启动时可通过 `lidar_model` 选择雷达型号，可选值为 `mid360` 和
`mid360s`：

```bash
ros2 launch fast_anchor_bringup fast_anchor_mid360.launch.py lidar_model:=mid360
ros2 launch fast_anchor_bringup fast_anchor_mid360.launch.py lidar_model:=mid360s
```

离线 rosbag 测试：

```bash
ros2 launch fast_anchor_bringup bag_localization.launch.py bag_path:=/path/to/bag
```

只启动 RViz：

```bash
ros2 launch fast_anchor_bringup rviz.launch.py
```

如果 `fast_anchor_mid360.launch.py` 无法 include 第三方 launch 文件，请检查当前环境中 FAST_LIO 和 livox_ros_driver2 的真实包名与 launch 文件名。

## LIO 后端

FastAnchor 的 LIO 前端可替换，通过 launch 参数 `lio_backend` 选择，默认 `fastlio2`：

```bash
# FastLIO2 后端（默认，行为与历史版本一致）
ros2 launch fast_anchor_bringup fast_anchor_mid360.launch.py lio_backend:=fastlio2

# yifanLIO 后端
ros2 launch fast_anchor_bringup fast_anchor_mid360.launch.py lio_backend:=yifanlio
```

非法取值会在 launch 阶段直接报错并停止，不会静默回退。

统一 LIO 接口（FastAnchor ICP 永远只订阅这两个话题）：

- `/Odometry`：`nav_msgs/msg/Odometry`，`camera_init -> body`（`body` 为 LIO IMU/机体系）。
- `/cloud_registered_body`：`sensor_msgs/msg/PointCloud2`，去畸变当前帧点云，位于 `body` 系。

FastLIO2 原生输出该接口；yifanLIO 输出 `/LIO/odom_imu`（`world -> IMU`）与
`/LIO/clouds_lidar`（`lidar` 系），由 `yifan_lio_adapter` 转换为统一接口。

配置文件：

- FastLIO2：`src/third_party/FAST_LIO/config/mid360_localization.yaml`
- yifanLIO：`src/third_party/lio/yaml/root_localization.yaml`（`mid360_localization.yaml`）
- yifanLIO adapter：`src/yifan_lio_adapter/config/yifan_lio_adapter.yaml`

详见 `docs/lio_backend.md`。

## 地图参数

默认配置位于：

```text
src/fast_anchor_bringup/config/fast_anchor_localization.yaml
```

启动时可以覆盖地图路径：

```bash
ros2 launch fast_anchor_bringup localization_only.launch.py \
  map_pcd_path:=/absolute/path/map_preprocessed.pcd \
  visualization_map_pcd_path:=/absolute/path/map_visualization.pcd \
  icp_map_pcd_path:=/absolute/path/map_preprocessed.pcd
```

## 主要输入话题

- `/Odometry`：FAST-LIO 输出的里程计。
- `/cloud_registered_body`：FAST-LIO 输出的当前帧注册点云。
- `/initialpose`：RViz/Nav2 发布的初始位姿。

## 默认输出话题

通过 `fast_anchor_bringup` 启动时，默认输出如下：

- `/fast_anchor/pose`
- `/fast_anchor/odom`
- `/fast_anchor/path`
- `/fast_anchor/status`
- `/fast_anchor/icp_result`
- `/fast_anchor/aligned_cloud`
- `/fast_anchor/local_map`
- `/fast_anchor/global_map`

## 推荐 TF 树

```text
map -> odom -> base_link -> livox_frame
```

职责划分：

- FAST-LIO 发布 `odom -> base_link`。
- FastAnchor 发布 `map -> odom`。
- `static_transform_publisher` 发布 `base_link -> livox_frame`。

不要让 FAST-LIO 和 FastAnchor 同时发布同一条 TF。

## 文档

- `AGENT.md`：给后续智能体读取的项目速览与约束说明。
- `docs/architecture.md`：系统架构说明。
- `docs/dependency_guide.md`：依赖说明。
- `docs/topic_interface.md`：话题接口说明。
- `docs/tf_tree.md`：TF 树说明。
- `docs/parameter_guide.md`：参数说明。
- `docs/troubleshooting.md`：常见问题排查。
