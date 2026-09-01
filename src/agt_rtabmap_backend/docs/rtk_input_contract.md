# RTK 输入合同（RTK_INPUT_CONTRACT）

> 状态：**未完成 - 等待设备信息**。按项目规则，RTK 融合的任何实现都必须在
> 下列输入项**全部确认**之后才开始；任何一项未知，禁止猜测并继续。
>
> 本文件是 Phase 7（双 RTK 融合）的输入合同与接入设计预留。

## 1. 目标接入位置

按 runtime 架构，RTK 融合属于 `agt_localization_fusion`（当前只有目录边界，
无业务实现，README 自述"预留 LIO、轮速、IMU、RTK、UWB 融合位置"）。

```text
RTAB-Map odom（或 FAST-LIVO2 odom）
Wheel odom（bunker_ros2）
IMU（/agt/sensors/imu/data）
双 RTK position（两个天线）
双 RTK heading（两天线基线向量）
          │
          ▼
  agt_localization_fusion   ← 融合实现落点
          │
          ▼
  连续融合状态估计
```

融合优先评估现成方案，**禁止从零手写 EKF**：

- `robot_localization`（EKF/UKF）：系统已装 `ros-humble-robot-localization` 3.5.4；
- `robot_localization/navsat_transform_node`：GPS/RTK 位置到全局 frame 的坐标转换；
- 双天线 heading：作为 yaw 观测输入 EKF，或先验证单天线位置 + 单天线航向
  的可行性。

## 2. 必须确认的输入项（全部未知前禁止编码）

### 2.1 设备与驱动

| 项 | 值 | 状态 |
|---|---|---|
| RTK 型号 | ？ | 未知 |
| 驱动 / 节点（如 nmea_navsat_driver / rtk 专用驱动） | ？ | 未知 |
| 驱动来源（apt / 源码 / vendor） | ？ | 未知 |

### 2.2 Topic 与消息

| 项 | 值 | 状态 |
|---|---|---|
| position topic（`sensor_msgs/NavSatFix` 或自定义） | ？ | 未知 |
| heading topic（`sensor_msgs/Imu` / `geometry_msgs/QuaternionStamped`） | ？ | 未知 |
| message type（如 NMEA 解析后） | ？ | 未知 |
| timestamp 来源（RTK 报文时间 / 接收机 PPS / ROS 接收时间） | ？ | 未知 |
| RTK FIX 状态字段（fixed / float / single 判定方式） | ？ | 未知 |
| position covariance（是否随 NavSatFix 提供） | ？ | 未知 |
| heading covariance（yaw 不确定性） | ？ | 未知 |

### 2.3 安装几何（`agt_description` 外参）

| 项 | 值 | 状态 |
|---|---|---|
| 天线 1（如 `rtk_left_link`）相对 `base_link` 外参 | ？ | 未知 |
| 天线 2（如 `rtk_right_link`）相对 `base_link` 外参 | ？ | 未知 |
| 基线长度（两天线间距） | ？ | 未知 |
| 天线朝向（哪个方向为基线正方向） | ？ | 未知 |
| 安装平面（车顶高度、左右对称性） | ？ | 未知 |

## 3. 数据流设计（草案，待确认后定稿）

```text
RTK 接收机
   ├── position topic（NavSatFix + covariance）
   ├── heading topic（双天线 yaw + covariance）
   └── status topic（FIX 质量门禁）
          │
          ▼
  agt_localization_fusion
     ├── 质量门禁：仅 FIX 状态下融合位置/航向
     ├── 外参：base_link <-> rtk_left/right_link（agt_description 参数化）
     ├── navsat_transform：经纬高 -> odom/map 直角坐标
     └── robot_localization EKF/UKF 融合 odom + IMU + RTK pos + RTK heading
          │
          ▼
  连续融合状态估计（输出合同待定，与 agt_mapping 输出互斥或作为增强源）
```

## 4. 验收前禁止事项

- 未确认设备型号/驱动/topic/message 前，禁止编写任何 RTK 相关代码；
- 未确认双天线基线几何前，禁止假设 `rtk_left_link`/`rtk_right_link` 外参；
- 禁止修改 `agt_localization_fusion` 以外的 runtime 模块来实现 RTK；
- 禁止用仿真伪造 RTK 数据替代真实设备验收。

## 5. 下一步

1. 获取 RTK 设备型号与驱动文档；
2. 实车/单机验证 position/heading/status 三个 topic 的内容与频率；
3. 填写本表格全部"未知"项；
4. 再进入融合实现（优先 robot_localization）。
