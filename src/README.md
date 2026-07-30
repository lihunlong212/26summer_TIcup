# D 题单机双路线飞行工程

## 启动

```bash
colcon build --symlink-install
source install/setup.bash

ROS_DOMAIN_ID=1 ros2 launch my_launch flight.launch.py
```

`flight.launch.py` 启动蓝海雷达、Cartographer、STM32 串口、面阵激光、AprilTag 相机、高度选择器、PID、任务控制器和 Domain 1/42 跨域桥，不启动 RViz、占据栅格或相机预览。跨域桥已包含在总启动中，不要再单独启动第二个桥进程。

[`my_launch/config/flight.yaml`](my_launch/config/flight.yaml) 保存跨域通信、PID、视觉控制阈值、任务高度、航点和高度源选择，不保存硬件串口、相机或雷达驱动参数。蓝海雷达沿用原工程的驱动启动与参数文件；STM32、面阵激光和相机使用各自代码默认值。Cartographer 的 `.lua` 属于算法配置资源。

若在 WSL 中构建，工作区路径不能包含中文或空格，否则 ROSIDL 可能错误拆分路径。部署到香橙派的纯英文路径不受影响。

## 任务选择和航点

工程启动后雷达、Cartographer、TF和高度节点先正常运行，可在等待任务期间完成建图。PID和STM32目标速度串口输出保持静默，只有任务控制器接受合法飞行模式后才放行；放行后的视觉丢失、投放和降落保持阶段仍按安全逻辑持续发送全零速度。

飞行模式可以来自STM32本地串口或配置的远端域（默认Domain 42）。STM32发送匿名协议V7帧：

```text
AA FF 11 01 mode SC1 SC2
```

其中`mode=1`为投放、`mode=2`为降落；其他值、错误长度或错误校验均忽略。示例完整帧为`AA FF 11 01 01 BC 84`和`AA FF 11 01 02 BD 85`。串口节点将合法模式发布到Domain 1的`/fly_choice`。

也可以在默认的 Domain 42 发布：

```bash
ROS_DOMAIN_ID=42 ros2 topic pub --once /fly_choice std_msgs/msg/UInt8 "{data: 1}"
```

- `1`：投放路线。发现 AprilTag 后先按配置在当前搜索高度视觉对准，再边对准边下降到 `55 cm`。到达后，两轴误差连续 3 个新帧都不超过 `100 px`，才发送一次 `0x11:[0x01]`。
- `2`：降落路线。发现 AprilTag 后先按配置在当前搜索高度视觉对准，再边对准边下降到配置值（当前为 `36 cm`）。实际高度到达阈值后不再要求视觉有效，立即发送一次 `0x44:[0x01]`；持续发送全零目标速度 5 秒，再发送一次 `0x44:[0x00]`。
- 其他值忽略；任务执行中的重复选择不会重置任务。
- STM32与远端域几乎同时选择时采用先到先执行，后到的选择在任务活动期间被忽略。

航点在 `flight.yaml` 中写成：

```yaml
route_drop_waypoints:
  - "(0 0 150 0)"
  - "(-32 65 150 0)"
  - "(-32 182 150 0)"
route_drop_normal_count: 1
```

四个数依次是 `(x_cm y_cm z_cm yaw_deg)`。前 `normal_count` 个是普通航点，剩余连续后缀均为搜索航点。四元组格式错误、包含非有限数值，或 `normal_count` 不小于航点总数时，控制器拒绝启动该任务。

飞机到达路线第一个起飞航点后，必须在位置、高度和偏航容差内连续稳定悬停 `2 s`，之后才前往第一个搜索航点。悬停期间 `/drone_state` 保持 `1`，不会接受视觉接管；漂出到达容差或 TF/高度无效时重新计时。

普通航段忽略 `/fine_data`。进入首个搜索航点时会清除旧视觉时间戳；搜索航点之间不等待，飞行途中收到新鲜 AprilTag 数据就立即裁剪剩余搜索航点并接管 XY。接管后先按 `pre_descent_alignment_sec` 保持当前搜索航点高度连续视觉对准，视觉丢失会重新计时；随后投放和降落分别把高度目标设为 `55 cm`、`36 cm`。实际高度不低于 `60 cm` 时最大下降速度为 `20 cm/s`，低于 `60 cm` 后降为 `9 cm/s`。

每个 `/fine_data` 只作为一个新视觉帧参与计数，但该帧对应的控制速度最多可以沿用 `0.2 s`。超过 `0.2 s` 未更新时，三帧计数清零，`/target_velocity` 持续精确输出 `[0,0,0,0]`；收到新帧后恢复控制。

投放完成后的返航顺序是 `(投放点x 投放点y 150 投放点yaw) → (0 0 150 0) → (0 0 0 0)`。降落停机 5 秒后发送起飞指令，发送成功时重新读取实时坐标，再执行 `(当前x 当前y 150 当前yaw) → (0 0 150 0) → (0 0 0 0)`。任务触发后不会再次接受视觉接管。

## AprilTag 接口

相机节点只识别 `DICT_APRILTAG_36h11`。内置默认值接受任意 ID；保存的调参文件可以改为只接受指定 ID。控制接口为：

- `/fine_data`：`std_msgs/msg/Int32MultiArray`，`[机体前后像素误差, 机体左右像素误差]`
- `/visual_takeover_active`：控制器是否已进入视觉接管
- `/vision_fresh`：控制器当前使用的视觉帧是否新鲜

色块识别、颜色选择和视觉模式切换接口均已删除。

## AprilTag 独立调参

调参时先停止 `flight.launch.py`，避免两个进程同时占用 `/dev/video0`，然后运行：

```bash
colcon build --symlink-install --packages-select apriltag_tuner_pkg
source install/setup.bash
ros2 run apriltag_tuner_pkg apriltag_tuner_node
```

调参节点不发布飞控话题。终端按键无需回车：`s` 保存、`r` 恢复默认值、`Space` 暂停/继续、`q` 或 `Esc` 退出。`s` 保存到 `~/.config/nezha/apriltag_detector.yaml`，正式 `drone_camera_node` 下次启动时自动读取；文件不存在或无效时会警告并使用内置参数。

## 高度和串口

PID、任务状态机与跨域状态统一使用 `/height`（`Int16`，厘米）。默认由面阵激光提供：`laser_array_pkg` 取 8×8 数据中的最高有效距离发布到 `/height_laser_array`，高度选择器再转发到 `/height`。

STM32 的 `0x05` 只发布 `/height_stm32`，默认不参与控制。如需切换，在 `flight.yaml` 修改：

```yaml
height_source: "stm32"  # 默认是 "laser_array"
```

STM32 串口还负责：

- 实际速度 `0x32`：建图产生 `map → laser_link` 后立即以 50 Hz 发送；XY 由定位坐标差分并转换到机体系，Z 由当前 `/height` 差分得到，三个分量均为厘米每秒的小端 `int16`，不受 `/fly_choice` 限制
- `/target_velocity`：任务控制接受 `/fly_choice=1/2`，并同时取得目标航点、`map → laser_link` TF 和 `/height` 后才开始计算和发送，编码为 `0x31` 的四个小端 `int16`
- `/serial_byte_command=[frame_id,value]`：发送单字节任务帧
- `/serial_byte_command_result=[frame_id,value,success]`：报告本地串口写入结果

任务帧仅在状态切换时请求一次。串口失败会进入 `ERROR` 并保持全零速度。

## 跨域坐标和状态

`flight.yaml` 默认配置 `local_domain_id: 1`、`remote_domain_id: 42`。Domain 42 的 `/drone_position` 类型为 `Float32MultiArray`，内容严格为 `[x_cm, y_cm]`，默认 10 Hz；只有本地 TF 可用时才发布。域ID、坐标话题、任务选择话题、状态话题和频率均可在同一配置段修改。

Domain 42 的 `/drone_state` 类型为 `UInt8`，整个工程只发送 `1~5`，不会发送 `0`：

- `1`：首次有效目标速度已经计算，正在飞往首个起飞航点
- `2`：搜索或伴飞；发现 Tag 后高度仍大于等于 `80 cm` 也保持该状态
- `3`：投放任务发现 Tag 且高度低于 `80 cm`
- `4`：降落任务发现 Tag 且高度低于 `80 cm`，包括停机、等待和解锁阶段
- `5`：投放完成、降落解锁成功或搜索结束后的返航阶段；最终落地后继续保持

首次有效目标速度产生前 `/drone_state` 完全静默。状态变化会立即转发，随后按配置频率持续发送。旧的 `/fleet/device_status` JSON 和 `/mission_state` 字符串话题均已删除。

桥只把 Domain 42 的 `/fly_choice` 转发到 Domain 1，并把坐标和 `/drone_state` 送回 Domain 42；Domain 1 的其他控制话题不会跨域暴露。

## 自动测试

以下测试不连接真实雷达、相机或 STM32：

```bash
colcon test --packages-select \
  activity_control_pkg pid_control_pkg drone_camera_pkg \
  apriltag_tuner_pkg domain_bridge_pkg
colcon test-result --verbose
```

它们覆盖航点配置拒绝、搜索段切换、投放三帧判定、投放/降落单次任务帧、实时坐标返航、视觉下降限速、持续全零速度、调参文件保存与加载、合成 AprilTag 检测，以及 Domain 1/42 双 Context 通信、厘米坐标和 `/drone_state` 状态码。
