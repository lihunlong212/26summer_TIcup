# Domain 42 地面站与无人机通信说明

## 1. 通信结构

飞行控制节点固定运行在无人机的 ROS 2 Domain 1，地面站固定使用 Domain 42。无人机上运行的 `domain_bridge` 同时加入两个 Domain，只转发任务需要的话题：

```text
地面站（Domain 42）                      无人机（Domain 1）

/fly_choice  UInt8
     ────────────────> domain_bridge ────────────────> /fly_choice

/drone_position Float32MultiArray
     <──────────────── domain_bridge <──────────────── TF(map→laser_link)

/drone_state UInt8
     <──────────────── domain_bridge <──────────────── 任务控制器

/tf、/tf_static TFMessage（2 Hz）
     <──────────────── domain_bridge <──────────────── 本地建图TF
```

Domain 42 不会直接看到 Domain 1 中的 PID、速度、激光高度等本地飞行话题。目标速度和实际速度仍由 Domain 1 内的节点通过串口发送给 STM32。

## 2. Domain 42 公开接口

### `/fly_choice`

- 方向：地面站 → 无人机
- 类型：`std_msgs/msg/UInt8`
- `1`：执行 AprilTag 跟随投放任务
- `2`：执行 AprilTag 跟随降落任务
- 其他数值：忽略

任务已经执行时，重复选择不会重置当前任务。建议地面站每次任务只发送一次。

### `/drone_position`

- 方向：无人机 → 地面站
- 类型：`std_msgs/msg/Float32MultiArray`
- 数据格式：`[x_cm, y_cm]`
- 单位：厘米
- 默认频率：10 Hz
- 坐标来源：Domain 1 中的 `map → laser_link` TF

示例：

```yaml
data:
- 87.5
- -37.5
```

表示无人机当前位置为 `x=87.5 cm，y=-37.5 cm`。此接口不包含高度和偏航角。

### `/drone_state`

- 方向：无人机 → 地面站
- 类型：`std_msgs/msg/UInt8`
- 默认频率：10 Hz
- 状态只使用 `1–5`，不会发送 `0`

| 数值 | 含义 |
|---:|---|
| 1 | 起飞或飞往首个起飞航点，PID 已经计算出有效目标速度 |
| 2 | 搜索或伴飞；发现 Tag 后实际高度仍大于等于 80 cm |
| 3 | 投放任务中，已发现 Tag 且实际高度低于 80 cm |
| 4 | 降落任务中，已发现 Tag 且实际高度低于 80 cm；包括停机、等待和解锁阶段 |
| 5 | 任务完成后返航，或搜索完未发现 Tag 后返航 |

未选择任务，或者 TF、高度等条件不足、PID 尚未产生有效目标速度时，无人机不会发布 `/drone_state`。因此刚启动时监听不到状态属于正常现象。

### `/tf` 和 `/tf_static`

- 方向：无人机 → 地面站
- 类型：`tf2_msgs/msg/TFMessage`
- 转发频率：2 Hz
- `/tf`：缓存并转发 Domain 1 每一对父子坐标系的最新动态变换
- `/tf_static`：缓存并转发 Domain 1 的静态变换，使用可靠、瞬态本地 QoS

当前 Cartographer 会产生动态 `/tf`。现有建图启动文件没有启动 `robot_state_publisher` 或额外静态 TF 节点，因此 Domain 1 当前不保证有 `/tf_static` 消息；桥已经保留该接口，以后本地出现静态变换时会自动转发。

## 3. 配置位置

跨域参数位于：

```text
src/my_launch/config/flight.yaml
```

当前配置为：

```yaml
domain_bridge:
  ros__parameters:
    local_domain_id: 1
    remote_domain_id: 42
    coordinate_topic: "/drone_position"
    fly_choice_topic: "/fly_choice"
    drone_state_topic: "/drone_state"
    publish_frequency_hz: 10.0
    tf_publish_frequency_hz: 2.0
```

修改 Domain ID 或话题名称后，重新构建相关包：

```bash
cd ~/26summer_TIcup
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select my_launch domain_bridge_pkg
source install/setup.bash
```

## 4. 无人机端启动

`flight.launch.py` 已经自动启动跨域桥，无人机端只需启动一次总工程：

```bash
cd ~/26summer_TIcup
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_DOMAIN_ID=1 ros2 launch my_launch flight.launch.py
```

该启动文件负责雷达定位、建图/定位、面阵激光高度、PID、任务控制、STM32 串口通信，并自动启动 `domain_bridge`。跨域桥内部会按配置创建 Domain 1 和 Domain 42 两套 ROS Context。

启动日志中应出现：

```text
bridge ready: local DOMAIN=1 <-> remote DOMAIN=42
```

不要再另外执行 `ros2 run domain_bridge_pkg domain_bridge`，否则会同时运行两个跨域桥，造成重复的坐标、状态和任务选择消息。

## 5. 地面站端准备

地面站和无人机必须连接到同一个局域网，推荐使用有线网络。两端尽量使用相同的 ROS 2 版本和 RMW 实现。

地面站每个终端都先执行：

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

无人机端也建议在启动前设置：

```bash
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

如果系统没有安装 `rmw_fastrtps_cpp`，不要强行设置该变量；但必须保证地面站和无人机选择兼容的 RMW。

## 6. 地面站命令行操作

### 检查是否发现无人机

```bash
ros2 topic list
ros2 topic info /fly_choice -v
ros2 topic info /drone_position -v
ros2 topic info /drone_state -v
```

正常情况下，Domain 42 应能看到：

```text
/fly_choice
/drone_position
/drone_state
/tf
/tf_static
```

监听跨域 TF：

```bash
ros2 topic hz /tf
ros2 topic echo /tf --once
ros2 topic echo /tf_static --once \
  --qos-reliability reliable \
  --qos-durability transient_local
```

### 监听无人机坐标

```bash
ros2 topic echo /drone_position
```

检查坐标发布频率：

```bash
ros2 topic hz /drone_position
```

### 监听无人机状态

`/drone_state` 使用可靠、瞬态本地 QoS。建议明确指定相同 QoS：

```bash
ros2 topic echo /drone_state \
  --qos-reliability reliable \
  --qos-durability transient_local
```

检查状态发布频率：

```bash
ros2 topic hz /drone_state
```

### 模拟选择投放任务

```bash
ros2 topic pub --once /fly_choice std_msgs/msg/UInt8 "{data: 1}"
```

### 模拟选择降落任务

```bash
ros2 topic pub --once /fly_choice std_msgs/msg/UInt8 "{data: 2}"
```

发送前应确认无人机已经完成定位、面阵激光高度有效、STM32 串口正常，并且周围具备安全试飞条件。

## 7. 地面站 Python 示例

下面的程序同时监听位置和状态，并在确认 `/fly_choice` 已经存在订阅者后发送一次任务选择。

保存为 `ground_station_example.py`：

```python
#!/usr/bin/env python3
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Float32MultiArray, UInt8


class GroundStation(Node):
    def __init__(self, fly_choice=None):
        super().__init__("domain42_ground_station")

        self.choice_pub = self.create_publisher(UInt8, "/fly_choice", 10)
        self.position_sub = self.create_subscription(
            Float32MultiArray,
            "/drone_position",
            self.on_position,
            10,
        )

        state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.state_sub = self.create_subscription(
            UInt8,
            "/drone_state",
            self.on_state,
            state_qos,
        )

        self.pending_choice = fly_choice
        self.choice_timer = self.create_timer(0.2, self.try_send_choice)

    def try_send_choice(self):
        if self.pending_choice is None:
            return
        if self.choice_pub.get_subscription_count() == 0:
            self.get_logger().info("等待发现无人机 /fly_choice 订阅者……")
            return

        msg = UInt8()
        msg.data = self.pending_choice
        self.choice_pub.publish(msg)
        self.get_logger().info(f"已发送 fly_choice={msg.data}")
        self.pending_choice = None
        self.choice_timer.cancel()

    def on_position(self, msg):
        if len(msg.data) < 2:
            self.get_logger().warning("收到格式错误的 /drone_position")
            return
        self.get_logger().info(
            f"位置：x={msg.data[0]:.1f} cm, y={msg.data[1]:.1f} cm"
        )

    def on_state(self, msg):
        names = {
            1: "起飞",
            2: "搜索/伴飞",
            3: "投放",
            4: "降落",
            5: "返航",
        }
        self.get_logger().info(
            f"状态：{msg.data} ({names.get(msg.data, '未知')})"
        )


def main():
    fly_choice = None
    if len(sys.argv) >= 2:
        fly_choice = int(sys.argv[1])
        if fly_choice not in (1, 2):
            raise SystemExit("fly_choice 只能是 1（投放）或 2（降落）")

    rclpy.init()
    node = GroundStation(fly_choice)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
```

只监听，不发送任务：

```bash
ROS_DOMAIN_ID=42 python3 ground_station_example.py
```

选择投放任务：

```bash
ROS_DOMAIN_ID=42 python3 ground_station_example.py 1
```

选择降落任务：

```bash
ROS_DOMAIN_ID=42 python3 ground_station_example.py 2
```

## 8. 单机联调方法

没有地面站时，也可以在无人机本机打开第三个终端模拟 Domain 42：

```bash
source /opt/ros/humble/setup.bash
source ~/26summer_TIcup/install/setup.bash
export ROS_DOMAIN_ID=42
ros2 topic echo /drone_position
```

再开一个 Domain 42 终端发送选择：

```bash
source /opt/ros/humble/setup.bash
source ~/26summer_TIcup/install/setup.bash
export ROS_DOMAIN_ID=42
ros2 topic pub --once /fly_choice std_msgs/msg/UInt8 "{data: 1}"
```

此时 Domain 1 的飞行终端应收到任务选择，完成有效 PID 计算后，Domain 42 才开始收到状态 `1`。

## 9. 常见故障排查

### Domain 42 完全看不到跨域话题

依次检查：

1. `flight.launch.py` 日志中是否出现 `bridge ready`。也可在 Domain 1 执行 `ros2 node list | grep domain_bridge`。
2. 两端 `ROS_DOMAIN_ID` 是否都为 `42`。
3. 两端 `ROS_LOCALHOST_ONLY` 是否为 `0`。
4. 两端是否处于同一网段，先用 `ping` 检查 IP 连通。
5. 两端 RMW 实现是否兼容。
6. 防火墙是否阻止 DDS 的 UDP 单播或组播。

修改环境变量后可重启 ROS 2 daemon：

```bash
ros2 daemon stop
ros2 daemon start
```

也可以测试局域网组播。在一台设备运行：

```bash
ros2 multicast receive
```

另一台设备运行：

```bash
ros2 multicast send
```

如果接收不到组播，需要检查交换机、路由器、无线 AP 的客户端隔离和系统防火墙。

### 能看到话题，但没有 `/drone_position`

这通常表示 Domain 1 尚未产生有效的 `map → laser_link` TF。检查：

```bash
ROS_DOMAIN_ID=1 ros2 run tf2_ros tf2_echo map laser_link
```

同时检查雷达定位和 Cartographer 是否正常运行。

### 能看到位置，但没有 `/drone_state`

启动后没有状态是设计行为。必须先满足以下条件：

- 收到合法的 `/fly_choice`；
- 已加载目标航点；
- TF 有效；
- 高度数据有效；
- PID 已经计算并发布第一帧合法目标速度。

只有以上条件满足后才会发布状态 `1`，建图阶段发送给 STM32 的实际速度不会触发状态发布。

### `/fly_choice` 已发布但飞机没有执行

检查 Domain 1 是否收到桥接后的选择：

```bash
ROS_DOMAIN_ID=1 ros2 topic echo /fly_choice
ROS_DOMAIN_ID=1 ros2 topic echo /fly_choice_status
```

再检查高度、TF、PID 和 STM32 串口节点日志。`/fly_choice_status` 是 Domain 1 内部放行信号，不需要转发到地面站。

## 10. 安全建议

- 首次联调时先拆除螺旋桨或可靠固定飞机，确认串口速度帧、任务选择和状态转换正确。
- 不要在任务执行中反复发布 `1` 和 `2`。
- 地面站失联不会改变 Domain 1 的本地闭环，任务状态机仍在无人机本地执行。
- 地面站只负责选择任务和观察状态，不应直接控制 Domain 1 的目标速度话题。
