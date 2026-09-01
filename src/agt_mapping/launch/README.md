# launch

建图链启动文件。

建图前用 `mapping_backend.launch.py` 选择连续里程计后端：

```bash
ros2 launch agt_mapping mapping_backend.launch.py backend:=fast_livo2
ros2 launch agt_mapping mapping_backend.launch.py backend:=rtabmap
```

两种模式都输出 `/agt/mapping/odometry`、`/agt/mapping/registered_points`
和 `odom -> base_footprint` TF。不要同时启动两个后端。

没有实车或 Gazebo 时，可先用轻量演示节点熟悉上位机检查流程：

```bash
ros2 launch agt_mapping mapping_backend_demo.launch.py backend:=fast_livo2
ros2 launch agt_mapping mapping_backend_demo.launch.py backend:=rtabmap
ros2 topic hz /agt/mapping/odometry
ros2 run tf2_ros tf2_echo odom base_footprint
```

带 Gazebo 空间感的上位机演示入口：

```bash
ros2 launch agt_mapping mapping_operator_sim.launch.py backend:=fast_livo2
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.35}, angular: {z: 0.25}}" -r 5
ros2 topic hz /agt/mapping/odometry
ros2 topic hz /agt/mapping/registered_points
```

停止后可换 RTAB-Map 风格后端重新启动：

```bash
ros2 launch agt_mapping mapping_operator_sim.launch.py backend:=rtabmap
```

该演示节点模拟的是上位机检查流程和 AGT mapping contract，不代表真实
FAST-LIVO2/RTAB-Map 算法精度。
