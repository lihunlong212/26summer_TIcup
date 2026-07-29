# 26summer TI Cup D 题飞行工作区

本工程实现两套无人机任务路线：跟随小车抛投、跟随小车动态降落。飞行闭环运行在
ROS Domain 1，任务选择与状态上报运行在 Domain 10。

详细接口、路线配置和调试方法见 [`src/README.md`](src/README.md)。

```bash
colcon build --symlink-install
source install/setup.bash

ROS_DOMAIN_ID=1 ros2 launch my_launch flight.launch.py
ROS_DOMAIN_ID=10 ros2 run domain_bridge_pkg domain_bridge --ros-args -p local_domain_id:=1
```
