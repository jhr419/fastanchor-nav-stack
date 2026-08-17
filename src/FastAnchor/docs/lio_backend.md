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
