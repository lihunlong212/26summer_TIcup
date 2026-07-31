# target_vision_pkg

备用视觉包，识别双圆环和中心十字。算法来自
`klayddd-beep/2026_H_vision` 的提交
`f56fe9bbd261e8ff7ac9083d1b5a034c04c6159f`，ROS接口已适配本工程。

## 飞控接口

- 发布 `/fine_data`，类型 `std_msgs/msg/Int32MultiArray`。
- 数据固定为 `[forward_error_px, lateral_error_px]`，与现有AprilTag节点一致。
- 原算法的 `[dx, dy]` 会转换为 `[-dy, -dx]`，避免飞控XY轴和符号错误。
- 目标无效时不发布旧数据，由任务控制器的 `fine_data_timeout_sec` 判定丢失。

节点监听 `/visual_descent_active` 和 `/height` 自动选择检测阶段：

1. 未开始下降：外圆、内圆和十字；
2. 开始下降且不低于低空阈值：内圆和十字；
3. 低于低空阈值：只识别十字。

`flight.yaml` 中 `visual_mode` 决定启动哪个视觉节点：

```yaml
vision_source:
  ros__parameters:
    visual_mode: "apriltag"  # apriltag或target
```

两个视觉节点不会同时启动，因此不会争抢摄像头。
