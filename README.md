# 26summer TI Cup D 题飞行工作区

本工程实现单架无人机的两套任务：跟随小车缓降投放，以及跟随小车动态降落。飞行闭环运行在 ROS Domain 1，飞行模式可由本地STM32串口或可配置远端域（默认Domain 42）的`/fly_choice`选择。建图得到定位后立即向STM32发送实际速度；收到合法模式后才计算并发送目标速度。

详细接口、配置和启动方法见 [`src/README.md`](src/README.md)。

AprilTag 参数可通过独立的 `apriltag_tuner_pkg` 实时调节，在终端按 `s` 保存后，
正式飞行相机节点会自动读取保存结果。

```bash
colcon build --symlink-install
source install/setup.bash

ROS_DOMAIN_ID=1 ros2 launch my_launch flight.launch.py
```
