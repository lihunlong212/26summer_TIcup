# AprilTag 独立调参工具

该包独立打开摄像头，不依赖飞行任务、PID 或 `drone_camera_pkg`，也不会发布
任何飞控话题。调参时必须先停止正式飞行启动文件，避免两个进程同时占用
`/dev/video0`。

构建并运行：

```bash
colcon build --symlink-install --packages-select apriltag_tuner_pkg
source install/setup.bash
ros2 run apriltag_tuner_pkg apriltag_tuner_node
```

建议直接使用 `ros2 run`，这样终端按键一定可用；也可以使用：

```bash
ros2 launch apriltag_tuner_pkg apriltag_tuner.launch.py
```

按键无需回车：

- `s`：保存到 `~/.config/nezha/apriltag_detector.yaml`
- `r`：恢复稳健默认值
- `Space`：暂停或继续图像
- `q`/`Esc`：退出

滑块可以调整目标 ID、阈值窗口、轮廓参数、角点细化、纠错率、反色检测、
CLAHE、锐化和模糊。绿色框是成功识别的 Tag，红点是当前选中 Tag，黄色框是
被拒绝的四边形候选。

正式 `drone_camera_node` 下次启动时会自动读取保存文件；不存在或内容无效时
会打印警告并使用内置默认参数。
