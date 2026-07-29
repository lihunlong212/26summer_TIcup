# D 题双路线飞行工程

## 启动

在工作区根目录构建并加载环境：

```bash
colcon build --symlink-install
source install/setup.bash
```

在 WSL 中构建时，工作区路径应避免中文和空格；Humble 的 ROSIDL 会把这类路径错误
拆分。部署到香橙派的普通英文路径不受影响。

分别启动本地飞行闭环和跨域桥：

```bash
ROS_DOMAIN_ID=1 ros2 launch my_launch flight.launch.py
ROS_DOMAIN_ID=10 ros2 run domain_bridge_pkg domain_bridge --ros-args -p local_domain_id:=1
```

`flight.launch.py` 启动蓝海雷达、robot_state_publisher、Cartographer、STM32
串口、面阵激光、高度来源选择器、PID 和任务控制器，不启动 RViz、占据栅格、相机或跨域桥。

## 路线选择

在 Domain 10 发布：

```bash
ROS_DOMAIN_ID=10 ros2 topic pub --once /route_choice std_msgs/msg/UInt8 "{data: 1}"
```

- `1`：跟随小车 5 秒后发送 `0x11:[0x01]`，然后返航降落。
- `2`：对准小车并以不低于 `-20 cm/s` 的速度缓降；低于 `45 cm` 时发送
  `0x44:[0x01]`，5 秒后发送 `0x44:[0x00]`，随后原地升高并返航。
- 其他值不会启动任务。

路线坐标及航点类型在
[`activity_control_pkg/config/routes.yaml`](activity_control_pkg/config/routes.yaml)
中配置。类型为 `1=NORMAL`、`2=SEARCH_DROP`、`3=SEARCH_LAND`。

## 视觉预留接口

本轮不启动视觉节点。后续视觉程序只需发布：

- 话题：`/fine_data`
- 类型：`std_msgs/msg/Int32MultiArray`
- `data[0]`：机体前后方向像素误差
- `data[1]`：机体左右方向像素误差

数据超过 `0.2 s` 未更新时，PID 直接输出
`/target_velocity=[0,0,0,0]`；抛投路线的连续 5 秒计时也会清零。
`drone_camera_pkg` 仅保留源码，等待后续视觉整合。

## 高度来源

飞行高度统一由 `/height` 提供，PID、航点状态机和 Domain 10 状态上报都只订阅该话题。
默认来源是面阵激光：`laser_array_pkg` 每帧取 64 束中的**最高有效距离**并发布
`/height_laser_array`（`Int16`，厘米）；高度选择器将它转发到 `/height`。

STM32 的 `0x05` 仅发布到 `/height_stm32`，默认不参与高度 PID。若需要临时切换，修改
[`my_launch/config/height_source.yaml`](my_launch/config/height_source.yaml) 中的：

```yaml
height_source: laser_array  # 可改为 stm32
```

重启 `flight.launch.py` 后生效。激光节点默认关闭 UART 辅助融合，避免 STM32 高度影响面阵激光高度。

## 串口接口

`uart_to_stm32` 默认使用 `/dev/ttyS6`、`921600`：

- 接收 `0x05`：两个小端有符号字节，发布 `/height_stm32`，类型 `Int16`，单位厘米；默认不参与控制。
- 发送 `0x31`：`/target_velocity` 的四个 `int16` 小端值。
- `/serial_byte_command=[frame_id,value]`：发送长度为 1 的通用任务帧。
- `/serial_byte_command_result=[frame_id,value,success]`：报告本地串口完整写入结果。

任务状态机只在状态切换时请求一次 `0x11` 或 `0x44`。写入失败会进入
`ERROR` 并保持全零速度；`0x44:[0x01]` 的停留计时从本地写入成功开始。

## 跨域状态

Domain 10 的 `/fleet/device_status` 为 JSON 字符串，默认 10 Hz，包含：

`device_id`、`x_cm`、`y_cm`、`z_cm`、`yaw_deg`、`route_choice`、
`current_waypoint_index`、`mission_state`、`vision_active`、`vision_fresh`。

只有 `/route_choice` 和上述状态被跨域桥转发；Domain 1 的控制话题不会暴露到
Domain 10。

## 保留包

- `activity_control_pkg`：双路线航点和任务状态机。
- `pid_control_pkg`：普通航点、视觉 XY、缓降限制及全零安全输出。
- `uart_to_stm32`、`serial_comm`：STM32 协议与串口。
- `laser_array_pkg`：面阵激光高度（64 束最大有效距离）及下方障碍信息。
- `bluesea2`、`my_carto_pkg`：雷达和 Cartographer 定位。
- `domain_bridge_pkg`：Domain 1/10 轻量桥。
- `my_launch`：Domain 1 总启动。
- `drone_camera_pkg`：暂存，当前不启动。

已删除旧 Action 调度、多无人机订单及第二架无人机相关代码。
