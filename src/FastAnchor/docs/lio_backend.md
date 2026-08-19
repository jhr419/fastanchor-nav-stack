# LIO Backend

FastAnchor 的 LIO 前端是可替换的。当前支持两个后端：

```text
                    ┌── FastLIO2 (fast_lio) ────────────────┐
LiDAR + IMU ────────┤                                      ├─> Common LIO Interface ─> FastAnchor ICP ─> Localization
                    └── yifanLIO (lio) + yifan_lio_adapter ┘
```

通过 launch 参数 `lio_backend` 选择，默认 `fastlio2`：

```bash
ros2 launch fast_anchor_bringup fast_anchor_mid360.launch.py lio_backend:=fastlio2
ros2 launch fast_anchor_bringup fast_anchor_mid360.launch.py lio_backend:=yifanlio
```

非法取值会在 launch 阶段直接报错并停止，不会静默回退。

## 统一 LIO 接口（FastAnchor ICP 的输入）

ICP 节点（`fast_anchor_localization_node`）永远只订阅两个话题，不感知后端：

| 接口话题 | 类型 | 语义 |
| -------- | ---- | ---- |
| `/Odometry` | `nav_msgs/msg/Odometry` | `camera_init -> body`，其中 `body` 为 LIO 的 IMU/机体系 |
| `/cloud_registered_body` | `sensor_msgs/msg/PointCloud2` | 去畸变后的当前帧点云，位于 `body`（IMU/机体）坐标系 |

- FastLIO2 原生输出这两个话题，无需适配。
- yifanLIO 原生输出 `/LIO/odom_imu`（`world -> IMU`）和 `/LIO/clouds_lidar`（`lidar` 系），
  由 `yifan_lio_adapter` 转换为上述统一接口。

yifanLIO 的原生接口不会被修改：`/LIO/odom_imu` 仍保持高频
`world -> 物理 IMU` 语义。adapter 只在 FastAnchor 公共接口上发布换算后的虚拟
body 位姿和点云，因此需要物理 IMU 位姿的模块仍可直接订阅原生话题。

## 从源码确认的数据链路

```text
/livox/lidar + /livox/imu
        │
        ├─ FastLIO2 ─> /Odometry + /cloud_registered_body ─┐
        │                                                   │
        └─ yifanLIO ─> /LIO/odom_imu + /LIO/clouds_lidar    │
                              │                              │
                     yifan_lio_adapter                       │
                              └──────────────────────────────┤
                                                             ▼
                           current body scan + same-stamp LIO pose
                                                             │
RViz /initialpose ─> T_map_odom initial guess ───────────────┤
existing PCD map (map frame) ────────────────────────────────┤
                                                             ▼
                                                      FastAnchor ICP
                                                             │
                                      T_map_odom = T_map_base(refined)
                                                   * inverse(T_odom_base)
```

FastAnchor 没有使用 `/cloud_registered` 世界系点云、LIO 累计地图或 yifanLIO
的 relocation 地图。ICP source 是**去畸变后的当前帧 body 点云**；ICP target
是 FastAnchor 从 `map.icp_pcd_path` 加载并降采样的已有 PCD 地图。

## 两种后端的实际语义对比

| 项目 | FastLIO2 | yifanLIO 原生 | yifan adapter 后公共接口 |
| ---- | -------- | ------------- | ------------------------- |
| LiDAR 输入 | `/livox/lidar` | `/livox/lidar` | 不处理传感器输入 |
| IMU 输入 | `/livox/imu` | `/livox/imu` | 不处理传感器输入 |
| odom | `/Odometry` | `/LIO/odom_imu` | `/Odometry` |
| odom pose | `T_camera_init_virtual_body` | `T_world_physical_imu` | `T_camera_init_virtual_body` |
| odom 频率 | scan 更新频率 | scan 后验 + 高频 IMU 预测 | 全部保留 |
| cloud | `/cloud_registered_body` | `/LIO/clouds_lidar` | `/cloud_registered_body` |
| cloud 内容 | 去畸变当前帧（非累计图） | 去畸变当前帧（非累计图） | 同左 |
| cloud frame | FastLIO 虚拟 body/IMU | 物理 lidar | FastLIO 虚拟 body |
| scan/pose 时间 | 同一 `lidar_end_time` | 后验 pose 与 scan 同一 `lidar_end_time` | cloud 等待精确同 stamp 后验 pose；FastAnchor 从高频缓存按 stamp 取 pose |
| TF | `camera_init -> body` | launch 中关闭原生 `world -> IMU` | `camera_init -> body`（高频） |
| 初始位姿/地图修正 | FastAnchor 统一处理 | 不使用 yifan relocation | FastAnchor 统一处理 |

## 已修复的不等价根因

工作正常的 FastLIO2 配置在估计前对 LiDAR 点和 IMU 向量同时应用：

```text
R_common_physical = Rz(0) * Ry(0.566395) * Rx(-0.034101)
```

所以 FastLIO2 的 `body` 是与机器人安装约定一致的虚拟 body，而不是未经处理的
物理 IMU 坐标。原 adapter 只做了 LiDAR→IMU 外参，随后把物理 IMU pose/cloud
直接改名为 `body`，约 32 度的安装倾角没有被消化；点坐标、odom pose 与
FastAnchor 的 `body_to_base` 假设不一致。

adapter 现在执行真实坐标变换：

```text
p_imu    = R_imu_lidar * p_lidar + t_imu_lidar
p_body   = R_body_imu * p_imu
T_W_body = T_W_imu * inverse(T_body_imu)
```

其中 `R_body_imu` 来自 adapter 参数 `imu_to_common_body_rpy`，必须与工作基线
`FAST_LIO preprocess.input_rotation_rpy_rad` 一致。pose 右乘逆旋转与点云左乘旋转
互为配套，保证 `T_W_body * p_body == T_W_imu * p_imu`；不是只修改 frame 名称。

第二个根因是 yifanLIO 保留高频 IMU 预测 odom，而旧 FastAnchor 在 cloud callback
中直接取“最新 odom”，可能拿到比 scan 更新的预测 pose。现在：

- adapter 继续逐条转发高频 odom，不丢失 yifanLIO 特性；
- adapter 仅在精确同 stamp 的 scan 后验 odom 已到达后发布对应 cloud；
- FastAnchor 为两个 backend 共用同一个 odom 时间缓存，并按 scan stamp 选择 pose；
- 找不到容差内 pose 时跳过该 scan，不用错误时间的 pose 启动 ICP。

## 后端输入话题

两个后端共用相同的传感器输入：

| 话题 | 类型 |
| ---- | ---- |
| `/livox/lidar` | `livox_ros_driver2/msg/CustomMsg` |
| `/livox/imu` | `sensor_msgs/msg/Imu` |

## 配置路径

| 后端 | 配置 |
| ---- | ---- |
| FastLIO2 | `src/third_party/FAST_LIO/config/mid360_localization.yaml`（launch 参数 `fastlio_config_file`） |
| yifanLIO | `src/third_party/lio/yaml/root_localization.yaml` -> `mid360_localization.yaml`（launch 参数 `yifanlio_config_path`） |
| yifanLIO adapter | `src/yifan_lio_adapter/config/yifan_lio_adapter.yaml`（launch 参数 `yifanlio_adapter_config`） |

## TF 职责

```text
map -> camera_init -> base_link -> livox_frame
                   \-> body
```

- FastAnchor（ICP）发布 `map -> camera_init` 与 `camera_init -> base_link`。
- FastLIO2 发布 `camera_init -> body`。
- yifanLIO 后端下，`camera_init -> body` 由 `yifan_lio_adapter` 发布（镜像 FastLIO2）；
  yifanLIO 自身的 TF（`world -> IMU` 等）通过 `publish_tf:=false` 关闭，避免重复/断裂。
- `base_link -> livox_frame` 由静态 TF 发布。

## 外参说明

两个后端的 LiDAR→IMU 外参遵循**相同的数学约定**：

```text
p_imu = R_lidar_imu * p_lidar + t_lidar_imu
```

即旋转/平移均为“IMU 系下描述、从 lidar 变换到 IMU”。

- FastLIO2：`mapping.extrinsic_T` / `mapping.extrinsic_R`（`mid360_localization.yaml`）。
- yifanLIO：`offset.imu_t_lidar` / `offset.imu_R_lidar`（`mid360_localization.yaml`）。

adapter 从 yifanLIO 的同一份 yaml 读取外参（单一事实来源），不复制、不猜测方向。
注意：两个工程的配置数值并不相同（对应不同的安装姿态假设），使用时需按实际安装
对齐各自配置，勿在未确认的情况下互相复制数值。

launch 会把 `yifanlio_config_path` 同时传给 yifanLIO 和 adapter，确保自定义 root
配置下话题名与外参仍来自同一份配置，不会出现 LIO 已切换配置而 adapter 仍读取
`root_localization.yaml` 的情况。
