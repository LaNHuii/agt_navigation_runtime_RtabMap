# agt_rtabmap_backend

RTAB-Map 3D-LiDAR SLAM 后端，作为 `agt_navigation_runtime` 中 FAST-LIVO2
后端**即插即用**的替换模块。

本包完全自包含：只依赖标准 ROS 2 消息接口与官方 `rtabmap_ros`
(Humble) 运行时，不依赖 runtime 的任何内部包。整个目录可以拷入
`agt_navigation_runtime/src/` 直接构建整合。

## 模块定位

```text
                    AGT 传感器接口
                          │
            ┌─────────────┴─────────────┐
            │                           │
            ▼                           ▼
     FAST-LIVO2 backend          RTAB-Map backend      (二选一)
            │                           │
            └─────────────┬─────────────┘
                          ▼
                  AGT 统一 mapping 接口
            /agt/mapping/odometry
            /agt/mapping/registered_points
                          │
                          ▼
                agt_localization（ICP/NDT 重定位 + GlobalCorrectionManager）
                          │
                          ▼
                        Nav2 ...
```

两个 backend 对外的公共接口完全一致，切换只发生在启动层：

```bash
# FAST-LIVO2
ros2 launch agt_mapping fast_livo2_mapping.launch.py ...

# RTAB-Map（本包）
ros2 launch agt_rtabmap_backend rtabmap_mapping.launch.py ...
```

## 公共接口合同（与 FAST-LIVO2 backend 相同）

| 类型 | Topic / 含义 | 消息类型 |
|---|---|---|
| 连续里程计 | `/agt/mapping/odometry`，`odom -> base_footprint` | `nav_msgs/Odometry` |
| 注册点云 | `/agt/mapping/registered_points`，当前帧在 `odom` 系 | `sensor_msgs/PointCloud2` |
| 里程计 TF | `odom -> base_footprint`（本包 adapter 是唯一发布者） | TF |

输入（全部参数化，无硬编码）：

| 输入 | Topic | 消息类型 |
|---|---|---|
| 3D 点云 | `/agt/sensors/lidar/points`（默认） | `sensor_msgs/PointCloud2` |
| 可选 IMU | `/agt/sensors/imu/data`（默认 dummy，禁用） | `sensor_msgs/Imu` |

## TF ownership（严格遵守 runtime 约束）

- `odom -> base_footprint`：**唯一发布者 = 本包 adapter**
  （`agt_mapping_rtabmap_adapter`），RTAB-Map 节点 `publish_tf=false`。
- `map -> odom`：**唯一发布者 = `agt_localization` 的
  GlobalCorrectionManager**。RTAB-Map 建图模式下 `map_frame_id == odom_frame_id`
  （不产生第二个全局系）；RTAB-Map 全局定位（Phase 5）将通过
  `/agt/localization/coarse_pose` 接入现有定位链，禁止直接发布 `map -> odom`。

## 依赖

- 构建：`rclpy`、`nav_msgs`、`sensor_msgs`、`geometry_msgs`、`tf2_ros`、
  `tf2_sensor_msgs`（全部为标准 Humble 包）。
- 运行：官方 `rtabmap_ros`（Humble）：
  ```bash
  sudo apt install ros-humble-rtabmap-ros
  # rosdep 键：rtabmap_odom / rtabmap_slam
  ```
- 传感器输入链路（`/agt/sensors/lidar/points`）由 `agt_sensor_adapters`
  的 `livox_custom_to_pointcloud2` 节点提供（CustomMsg -> PointCloud2），
  已在 `agt_sensor_adapters` 中实现：

  ```bash
  ros2 launch agt_sensor_adapters livox_points.launch.py
  # 默认输入 /agt/sensors/lidar/custom_filtered（自滤除后），
  # 输出 /agt/sensors/lidar/points
  ```

## 启动（建图模式）

```bash
# 完整链路：驱动 + 自滤除 + 标准点云
ros2 launch agt_sensor_adapters mid360.launch.py
ros2 launch agt_sensor_adapters lidar_self_filter.launch.py
ros2 launch agt_sensor_adapters livox_points.launch.py

# RTAB-Map mapping（icp_odometry + rtabmap + AGT adapter）
ros2 launch agt_rtabmap_backend rtabmap_mapping.launch.py \
  lidar_topic:=/agt/sensors/lidar/points \
  database_path:=runtime/maps/rtabmap/rtabmap.db
```

关键 launch 参数（全部参数化，机器人相关不写死）：

| 参数 | 默认 | 说明 |
|---|---|---|
| `lidar_topic` | `/agt/sensors/lidar/points` | 标准点云输入 |
| `frame_id` | `lidar_link` | RTAB-Map 后端 body frame（雷达系） |
| `base_frame` | `base_footprint` | AGT 里程计 child frame |
| `odom_frame` | `odom` | 连续里程计世界系 |
| `map_frame` | `odom` | 建图模式下 map==odom |
| `database_path` | `runtime/maps/rtabmap/rtabmap.db` | RTAB-Map 数据库 |
| `imu_topic` | `/rtabmap/no_imu` | 显式设置才启用 IMU |
| `publish_backend_tf` | `false` | **必须保持 false** |

## 节点组成

1. `rtabmap_odometry_backend`（`rtabmap_odom/icp_odometry`）
   - 标准 PointCloud2 -> ICP 里程计，输出 `/rtabmap/odom`；
   - `publish_tf=false`，不发布任何 TF。
2. `rtabmap_mapping`（`rtabmap_slam/rtabmap`）
   - 消费 `/rtabmap/odom` 与点云，做增量建图 + 回环检测/图优化；
   - `map_frame_id==odom`（建图模式），`publish_tf=false`；
   - 数据库保存回环图，供 Phase 5 定位复用。
3. `agt_mapping_rtabmap_adapter`（`scripts/rtabmap_adapter.py`）
   - 把 `/rtabmap/odom`（body=lidar_link）经 `base -> lidar_link` 外参转换为
     `base_footprint`，发布 `/agt/mapping/odometry` 与 `odom -> base_footprint` TF；
   - 把当前帧点云经 TF 变换到 `odom` 系，发布
     `/agt/mapping/registered_points`（保留原始时间戳）。

## 与 FAST-LIVO2 的差异（已知）

- FAST-LIVO2 融合 IMU，RTAB-Map ICP 里程计无 IMU 初值：快速旋转或运动退化
  场景下 RTAB-Map odom 漂移更大，属于后端特性差异，不是接口差异。
- `registered_points` 语义一致：均为"当前帧在 odom 系下的点云"。
- 建图质量：RTAB-Map 依赖回环闭合修正累积漂移；无回环的长直场景与
  FAST-LIVO2 相当或略差，需 Phase 6 同 bag 对比验证。

## 整合到 agt_navigation_runtime

1. 将整个 `src/agt_rtabmap_backend/` 拷入 runtime 的 `src/`。
2. `agt_sensor_adapters` 中的转换节点与 `livox_points.launch.py` 已就位
   （如整合的 runtime 版本没有，请一并拷入 `src/agt_sensor_adapters/`）。
3. 安装运行时依赖：`sudo apt install ros-humble-rtabmap-ros`。
4. `colcon build --symlink-install`，然后按上面方式切换 backend 启动。

不修改 FAST-LIVO2、localization、Nav2、safety、chassis 的任何代码。

## 阶段状态

- [x] P1：最小 backend（launch/config/dependency）独立启动
- [x] P2：标准 PointCloud2 输入（`agt_sensor_adapters` 转换节点）
- [x] P3：RTAB-Map -> AGT mapping 合同 adapter
- [x] P4：与 FAST-LIVO2 二选一切换（本包即切换的 RTAB 侧）
- [ ] P5：RTAB-Map 全局定位/回环接入 `/agt/localization/coarse_pose`
      （配置骨架见 `config/rtabmap_localization.yaml`，未启用）
- [ ] P6：同一 rosbag 对比 FAST-LIVO2 与 RTAB-Map（需数据）
- [ ] P7：双 RTK 融合（输入合同见 `docs/rtk_input_contract.md`，未实现）

## 验证记录

- 单元测试：`test/test_rtabmap_adapter.py`（7 项，位姿/外参/速度变换数学），
  `colcon test` 全部通过。
- 端到端冒烟（合成数据，无硬件）：
  - `icp_odometry` + `rtabmap` + adapter 全链路启动；
  - `/rtabmap/odom` 10 Hz 稳定；
  - `/agt/mapping/odometry` 输出 `odom -> base_footprint` 合同正确；
  - `/agt/mapping/registered_points` 当前帧变换到 `odom` 系正确
    （3300 点、保留时间戳）；
  - `odom -> base_footprint` TF 唯一发布，外参补偿正确（lidar 安装高
    0.4 m -> TF z=-0.4 m）。
- 转换节点：合成 Livox `CustomMsg` -> `/agt/sensors/lidar/points`
  （400 点、10 Hz、frame `livox_frame`）验证通过。
- 已知环境注意：本 workspace 位于中文路径 `桌面`，CMake 3.22 的 rosidl
  生成在非 ASCII build 路径下会失败（`list index out of range`）；构建
  `livox_ros_driver2` 等消息包需使用 ASCII build 目录（本仓库安装产物已就绪，
  见 `.rtabmap_local/` 与 `install/`）。整合进 runtime 时若 workspace 为
  ASCII 路径则无此问题。
- 待实机/bag 验证：里程计连续性、回环、地图质量、CPU/RAM/频率（Phase 6）。
