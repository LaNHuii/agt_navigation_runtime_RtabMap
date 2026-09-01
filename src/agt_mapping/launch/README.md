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
