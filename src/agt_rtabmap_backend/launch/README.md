# agt_rtabmap_backend launch 说明

## rtabmap_mapping.launch.py — 建图模式（与 FAST-LIVO2 对等）

启动三个节点：

1. `rtabmap_odometry_backend`（`rtabmap_odom/icp_odometry`）
   - 标准 `PointCloud2` -> ICP 里程计，输出 `backend_odom_topic`（默认
     `/rtabmap/odom`）；
   - `publish_tf=false`：RTAB-Map 不发布任何 TF。
2. `rtabmap_mapping`（`rtabmap_slam/rtabmap`）
   - 消费 backend odom + 点云，增量建图与回环图优化；
   - `map_frame_id == odom_frame_id`（建图模式不产生第二个全局系）；
   - 数据库写入 `database_path`。
3. `agt_mapping_rtabmap_adapter`（本包 `scripts/rtabmap_adapter.py`）
   - `/rtabmap/odom` -> `/agt/mapping/odometry`（`odom -> base_footprint`，
     含唯一 TF 发布）；
   - 当前帧点云 -> `/agt/mapping/registered_points`（`odom` 系，保留时间戳）。

示例：

```bash
# 输入链路（sensor 侧由 agt_sensor_adapters 提供）
ros2 launch agt_sensor_adapters mid360.launch.py
ros2 launch agt_sensor_adapters lidar_self_filter.launch.py
ros2 launch agt_sensor_adapters livox_points.launch.py

# RTAB-Map 建图
ros2 launch agt_rtabmap_backend rtabmap_mapping.launch.py \
  lidar_topic:=/agt/sensors/lidar/points \
  database_path:=runtime/maps/rtabmap/rtabmap.db \
  use_sim_time:=false
```

## 定位模式（Phase 5 预留）

`rtabmap_localization.launch.py` 尚未创建。定位模式的设计约束见
`config/rtabmap_localization.yaml` 与 `../README.md`：RTAB-Map 的全局位姿
必须经 `/agt/localization/coarse_pose` 进入现有定位链，禁止直接发布
`map -> odom`。
