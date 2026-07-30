# Domain 42 地面站接口与处理说明

本文档供地面站程序开发使用。飞机飞行闭环运行在 ROS 2 Domain 1，飞机上的跨域桥会在 Domain 42 发布坐标、任务状态和 TF，并监听地面站发送的飞行任务选择。

## 1. 通信方向

```text
地面站（Domain 42）                         飞机

/fly_choice
    UInt8：1=投放，2=降落
        ───────────────────────────────────>

/drone_position
    Float32MultiArray：[x_cm, y_cm]
        <───────────────────────────────────

/drone_state
    UInt8：1～5
        <───────────────────────────────────

/tf、/tf_static
    TFMessage
        <───────────────────────────────────
```

地面站只需要加入 Domain 42，不要加入飞机内部使用的 Domain 1。

## 2. 话题汇总

| 话题 | 方向 | ROS 2 类型 | 频率 | 用途 |
|---|---|---|---:|---|
| `/fly_choice` | 地面站 → 飞机 | `std_msgs/msg/UInt8` | 单次发送 | 选择投放或降落任务 |
| `/drone_position` | 飞机 → 地面站 | `std_msgs/msg/Float32MultiArray` | 10 Hz | 显示飞机平面坐标 |
| `/drone_state` | 飞机 → 地面站 | `std_msgs/msg/UInt8` | 10 Hz | 显示飞机当前任务阶段 |
| `/tf` | 飞机 → 地面站 | `tf2_msgs/msg/TFMessage` | 2 Hz | 动态坐标系，用于地图或姿态显示 |
| `/tf_static` | 飞机 → 地面站 | `tf2_msgs/msg/TFMessage` | 2 Hz，有数据时发布 | 静态坐标系 |

Domain 42 不再提供 `/mission_state` 或 `/fleet/device_status`。地面站不能依赖这两个旧话题。

## 3. 飞机坐标 `/drone_position`

### 消息定义

```text
话题：/drone_position
类型：std_msgs/msg/Float32MultiArray
数据：[x_cm, y_cm]
单位：厘米
坐标系：map
频率：10 Hz
```

示例消息：

```yaml
data:
- 87.5
- -37.5
```

地面站应解释为：

```text
x = 87.5 cm
y = -37.5 cm
```

处理要求：

1. 收到消息后先确认 `data` 至少有两个元素。
2. `data[0]` 显示为 X 坐标，`data[1]` 显示为 Y 坐标。
3. 坐标已经是厘米，地面站不需要乘以 100。
4. 该消息不包含高度和偏航角。
5. 如果需要完整姿态，可从 `/tf` 中查询 `map → laser_link`。
6. 建图尚未产生有效 `map → laser_link` TF 时，飞机不会发布坐标。

建议地面站保存最后一次坐标接收时间。任务运行中超过 1 秒没有新坐标，可将坐标显示标记为“数据超时”，但不要自动发送新的任务选择。

## 4. 飞行状态 `/drone_state`

### 消息定义

```text
话题：/drone_state
类型：std_msgs/msg/UInt8
数据：1～5
频率：10 Hz
QoS：Reliable + Transient Local
```

状态含义：

| 状态值 | 地面站显示建议 | 实际含义 |
|---:|---|---|
| 1 | 起飞 | PID 已经产生有效目标速度，正在飞往普通起飞航点 |
| 2 | 搜寻/伴飞 | 正在搜索小车、进行高空视觉对准，或识别 Tag 后高度仍不低于 80 cm |
| 3 | 投放 | 投放路线已经识别 Tag，且高度低于 80 cm；持续到投放成功 |
| 4 | 降落 | 降落路线已经识别 Tag，且高度低于 80 cm；包含关锁、停留5秒和等待解锁 |
| 5 | 返航 | 投放完成、降落重新起飞或搜索无结果后返航；最终落地后继续保持5 |

处理要求：

1. 地面站只处理 `1～5`，其他数值显示为“未知状态”。
2. 飞机永远不会发送状态 `0`。
3. 飞机刚启动、尚未选择任务或 PID 尚未产生第一帧有效目标速度时，可能完全没有 `/drone_state` 消息，这是正常现象。
4. `/drone_state` 使用瞬态本地 QoS。地面站重新连接后，会立即收到飞机最后保留的状态。
5. 地面站应以收到的最新状态覆盖界面，不要自行推测状态跳转。

推荐的界面映射：

```text
1 → 起飞
2 → 搜寻/伴飞
3 → 投放
4 → 降落
5 → 返航
无消息 → 等待任务状态
```

## 5. 任务选择 `/fly_choice`

### 消息定义

```text
话题：/fly_choice
类型：std_msgs/msg/UInt8
方向：地面站发布，飞机订阅
```

| 数值 | 任务 |
|---:|---|
| 1 | 投放任务 |
| 2 | 降落任务 |

处理要求：

1. 操作员确认后只发送一次。
2. 不要以10 Hz连续发布任务选择。
3. 其他数值无效，飞机会忽略。
4. 飞机任务已经执行时，重复的任务选择不会重置任务。
5. 发布前最好确认 `/fly_choice` 已经存在订阅者，防止 DDS 尚未完成发现时丢失第一次消息。

投放任务命令：

```bash
ros2 topic pub --once /fly_choice std_msgs/msg/UInt8 "{data: 1}"
```

降落任务命令：

```bash
ros2 topic pub --once /fly_choice std_msgs/msg/UInt8 "{data: 2}"
```

## 6. TF 处理

如果地面站只显示二维坐标，直接使用 `/drone_position` 即可，不需要解析 TF。

如果地面站需要在地图中显示飞机姿态：

- 订阅 `/tf`，类型为 `tf2_msgs/msg/TFMessage`。
- 订阅 `/tf_static`，类型同样为 `tf2_msgs/msg/TFMessage`。
- 查询 `map → laser_link` 得到飞机当前 X、Y 和偏航角。
- `/tf` 使用 Best Effort + Volatile QoS。
- `/tf_static` 使用 Reliable + Transient Local QoS。

飞机当前建图节点会发布动态 `/tf`。如果 Domain 1 没有静态 TF 发布者，Domain 42 的 `/tf_static` 会存在话题端点，但可能暂时没有消息。

## 7. 地面站环境配置

地面站每个终端启动前执行：

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
```

如果飞机明确使用 Fast DDS，地面站也设置：

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

飞机和地面站需要：

- 位于同一个局域网；
- 能够互相 `ping`；
- 使用兼容的 ROS 2 和 RMW；
- 防火墙允许 DDS 使用的 UDP 单播和组播；
- 无线网络没有启用客户端隔离。

修改 Domain ID 后建议重启 ROS 2 daemon：

```bash
ros2 daemon stop
ros2 daemon start
```

## 8. 命令行联调

检查话题：

```bash
ros2 topic list
```

正常应看到：

```text
/drone_position
/drone_state
/fly_choice
/tf
/tf_static
```

检查接口类型和端点：

```bash
ros2 topic info /drone_position -v
ros2 topic info /drone_state -v
ros2 topic info /fly_choice -v
```

监听坐标：

```bash
ros2 topic echo /drone_position
```

监听状态：

```bash
ros2 topic echo /drone_state \
  --qos-reliability reliable \
  --qos-durability transient_local
```

检查频率：

```bash
ros2 topic hz /drone_position
ros2 topic hz /drone_state
ros2 topic hz /tf
```

## 9. Python 地面站示例

下面的节点可以直接接收坐标和状态，并在确认飞机订阅者存在后发送一次任务选择。

保存为 `domain42_ground_station.py`：

```python
#!/usr/bin/env python3
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Float32MultiArray, UInt8


STATE_TEXT = {
    1: "起飞",
    2: "搜寻/伴飞",
    3: "投放",
    4: "降落",
    5: "返航",
}


class GroundStation(Node):
    def __init__(self, pending_choice=None):
        super().__init__("domain42_ground_station")

        self.last_position_time = None
        self.last_state_time = None
        self.pending_choice = pending_choice

        self.choice_pub = self.create_publisher(
            UInt8,
            "/fly_choice",
            10,
        )
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

        self.choice_timer = self.create_timer(0.2, self.try_send_choice)
        self.watchdog_timer = self.create_timer(0.5, self.check_timeout)

    def on_position(self, msg):
        if len(msg.data) < 2:
            self.get_logger().warning(
                "/drone_position格式错误，至少需要两个元素"
            )
            return

        self.last_position_time = time.monotonic()
        x_cm = float(msg.data[0])
        y_cm = float(msg.data[1])

        # 地面站界面在这里更新飞机坐标。
        self.get_logger().info(
            f"坐标：x={x_cm:.1f} cm, y={y_cm:.1f} cm"
        )

    def on_state(self, msg):
        self.last_state_time = time.monotonic()
        state = int(msg.data)
        text = STATE_TEXT.get(state, "未知状态")

        # 地面站界面在这里更新任务状态。
        self.get_logger().info(f"状态：{state}，{text}")

    def try_send_choice(self):
        if self.pending_choice is None:
            return

        if self.choice_pub.get_subscription_count() == 0:
            self.get_logger().info("等待发现飞机/fly_choice订阅者")
            return

        msg = UInt8()
        msg.data = self.pending_choice
        self.choice_pub.publish(msg)
        self.get_logger().info(
            f"已单次发送/fly_choice={self.pending_choice}"
        )
        self.pending_choice = None
        self.choice_timer.cancel()

    def check_timeout(self):
        now = time.monotonic()

        if (
            self.last_position_time is not None
            and now - self.last_position_time > 1.0
        ):
            self.get_logger().warning("飞机坐标超过1秒未更新")
            self.last_position_time = None

        if (
            self.last_state_time is not None
            and now - self.last_state_time > 1.0
        ):
            self.get_logger().warning("飞机状态超过1秒未更新")
            self.last_state_time = None


def main():
    choice = None
    if len(sys.argv) >= 2:
        choice = int(sys.argv[1])
        if choice not in (1, 2):
            raise SystemExit("任务选择只能是1（投放）或2（降落）")

    rclpy.init()
    node = GroundStation(choice)
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

只监听飞机：

```bash
ROS_DOMAIN_ID=42 python3 domain42_ground_station.py
```

发送投放任务并继续监听：

```bash
ROS_DOMAIN_ID=42 python3 domain42_ground_station.py 1
```

发送降落任务并继续监听：

```bash
ROS_DOMAIN_ID=42 python3 domain42_ground_station.py 2
```

## 10. 地面站界面建议

建议至少显示：

```text
连接状态：已连接 / 数据超时
飞机坐标：X cm、Y cm
任务状态：起飞 / 搜寻伴飞 / 投放 / 降落 / 返航
任务按钮：投放任务、降落任务
```

按钮处理建议：

1. 操作员点击投放或降落按钮。
2. 弹出确认提示。
3. 检查 `/fly_choice` 订阅者数量大于0。
4. 发布一次 `UInt8`。
5. 禁用两个任务按钮，防止任务过程中重复选择。
6. 收到 `/drone_state` 后更新状态显示。

地面站不应根据坐标或超时自动发布 `/fly_choice`，任务选择必须由操作员明确触发。
