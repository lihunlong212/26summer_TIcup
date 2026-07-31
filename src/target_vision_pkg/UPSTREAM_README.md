# 2026 H Vision - Orange Pi 5 Max ROS2 Target Detector

面向香橙派 5 Max 的单文件 ROS2 视觉节点。通过 USB UVC 摄像头识别两个同心圆环和中心十字，为无人机跟踪、抛投及动态着陆提供图像中心误差。

节点默认使用 V4L2、640x480、30 FPS 和 MJPG，开启检测画面窗口。窗口只显示摄像头原图、最终选中的圆环与十字、图像中心、目标中心和运行状态，不显示候选线段、二值图、边缘图或调参面板。

## 功能

- `HIGH_TARGET`：双圆识别并使用中心十字复核，适合完整标靶可见时。
- `TRANSITION`：只要求物理内圆环和中心十字同时有效，外圆允许已经超出视野。
- `LOW_CROSS`：先把粗笔画的两条平行边配对为中心线，再求两条近似正交中心线的交点；允许圆环部分或全部超出视野。
- 所有蓝色圆框都吸附到黑色圆环的外侧边缘，不在同一条黑线的内外边缘之间跳动。
- 新摄像头将圆环与十字交点拍得很清晰、导致完整圆轮廓断开时，会以已验证十字中心为唯一圆心扫描 30/50 双半径边缘；不会在全画面放宽圆置信度。
- 连续多帧确认、中心/半径跳变拒绝、短时丢失保持和 EMA 平滑。
- 只在目标有效时发布中心误差，失效时不会发布旧坐标冒充当前结果。
- 模式只接受无人机状态机命令，不会由视觉结果自行切换。按 `1→2→3` 切换时会继承已确认中心，Mode 3 仍必须在当前帧重新确认粗十字。
- ROS2 参数可在启动时或运行时调整，无需单独 YAML 文件。

## 文件

```text
vision_target.py    单文件视觉算法、V4L2 采集、ROS2 节点和可选画面显示
demo1.py            Windows 原四宫格与滑块调参入口
demo2.py            Windows 香橙派逻辑模拟入口（单检测画面）
vision_config.yaml  Demo 1 的可保存调参配置
README.md           部署、接口和调试说明
```

## Windows Demo

两个 Windows 入口与 `vision_target.py` 放在同一目录，共用同一套识别核心，避免复制算法后出现参数和逻辑不一致。

原 Windows 四宫格调参版本现在使用：

```powershell
python demo1.py --camera 1 --config vision_config.yaml
```

香橙派运行逻辑的 Windows 验证版本使用：

```powershell
python demo2.py --camera 1
```

若要保存不含圆、十字和文字叠加的摄像头画面，用于离线复现：

```powershell
python demo2.py --camera 1 --record-raw captures\mode3_raw.mp4
```

录像文件只写入摄像头帧，窗口仍显示实时检测结果。`.mp4` 使用 `mp4v`；路径以 `.avi` 结尾时使用 `MJPG`。

模式 3 的线段组合数量可在 Windows 上直接调整：

```powershell
python demo2.py --camera 1 --max-segments 80
```

`max-segments` 越低速度越快，但过低可能在复杂背景中同时删掉目标十字线段。建议先使用 `80`；需要更高速度时试 `60`，目标线段偶尔缺失时改为 `100`。

`demo2.py` 默认从 `OFF` 启动，窗口只显示最终选中的圆、十字、目标中心和状态。按键与 ROS2 模式值完全对应：

| 按键 | 模式 |
|---:|---|
| `0` | OFF |
| `1` | HIGH_TARGET |
| `2` | TRANSITION |
| `3` | LOW_CROSS |
| `Q` / `Esc` | 退出 |

终端中的 `vision_status` 对应 ROS2
`/land_air/vision_status`；`fine_data` 对应 `/fine_data`。无效状态下
`fine_data` 显示为 `null`，表示香橙派节点不会发布旧误差。

## Demo 2 与香橙派 ROS2 的对应关系

`demo2.py` 不是另一套简化算法。它和香橙派入口
`python3 vision_target.py --ros2` 共同调用 `vision_target.py` 中的：

- 同一份 `embedded_config()` 默认参数；
- 同一个 `TargetDetector`；
- 同一个 `detect_commanded_frame()` 模式分派；
- 同一套 `1→2→3` 跟踪交接、有效性和失效处理；
- 同一个单画面 `draw_overlay()` 显示。

两端只替换操作系统和通信接口：

| 功能 | Windows Demo 2 | Orange Pi 5 Max ROS2 |
|---|---|---|
| USB 摄像头 | DirectShow，通常 `--camera 1` | V4L2，使用实际 `/dev/videoN` 的 N |
| 模式命令 | 键盘 `0/1/2/3` | `/land_air/vision_mode`，`UInt8` |
| 有效误差 | 终端 JSON `fine_data` | `/fine_data`，`Int32MultiArray` |
| 识别状态 | 终端 JSON `vision_status` | `/land_air/vision_status` |
| 调试画面 | 单个处理后画面 | 同一个单处理画面，`show_dashboard:=true` |
| 原始录像 | 可选 `--record-raw`，仅用于 Windows 采样 | 不参与飞行逻辑，按需在上机调试时另行采集 |

因此以后调整 Mode 1/2/3 检测核心时，不需要再复制一份到香橙派。
Windows 验证通过的逻辑会直接进入 ROS2 节点。需要注意：Windows 的
`camera 1` 不代表 Linux 一定是 `/dev/video1`，香橙派必须以
`v4l2-ctl --list-devices` 的枚举结果为准。

## 环境要求

- Orange Pi 5 Max
- Ubuntu 22.04/24.04 或其他已正确安装 ROS2 的 Linux 系统
- ROS2 Humble/Jazzy 使用的 `rclpy`
- Python 3
- OpenCV 4
- NumPy
- `std_msgs`
- USB UVC 免驱摄像头

若系统已安装 ROS2，可补齐依赖：

```bash
sudo apt update
sudo apt install -y python3-opencv python3-numpy v4l-utils \
  ros-${ROS_DISTRO}-rclpy ros-${ROS_DISTRO}-std-msgs
```

确认用户具有摄像头权限：

```bash
sudo usermod -aG video "$USER"
```

重新登录后检查摄像头：

```bash
v4l2-ctl --list-devices
v4l2-ctl -d /dev/video0 --list-formats-ext
```

## 运行

首次下载或更新仓库：

```bash
git clone https://github.com/klayddd-beep/2026_H_vision.git
cd 2026_H_vision
git pull origin main
```

先加载 ROS2 环境：

```bash
source /opt/ros/${ROS_DISTRO}/setup.bash
```

直接启动单文件节点：

```bash
python3 vision_target.py --ros2
```

节点安全地从 `OFF` 模式启动，不会在控制端尚未指定视觉阶段时发布误差。启动后由无人机控制节点发布 `/land_air/vision_mode`。

香橙派中的阶段逻辑与 Demo 2 完全相同：

```text
0 = OFF
1 = 外圆外沿 + 内圆外沿 + 十字
2 = 内圆外沿 + 十字（外圆允许出界）
3 = 仅十字（两个圆都允许出界）
```

视觉节点不会根据画面自行改变 `1/2/3`，只能由无人机状态机发送模式。

指定摄像头和画面显示：

```bash
python3 vision_target.py --ros2 --ros-args \
  -p camera.index:=0 \
  -p camera.width:=640 \
  -p camera.height:=480 \
  -p camera.fps:=30 \
  -p show_dashboard:=true
```

默认会打开 `UAV Target Vision - ROS2` 窗口。按 `Q` 或 `Esc` 关闭节点。

纯后台或没有桌面环境时必须关闭窗口：

```bash
python3 vision_target.py --ros2 --ros-args -p show_dashboard:=false
```

## ROS2 接口

### 识别模式输入

```text
/land_air/vision_mode    std_msgs/msg/UInt8
```

| 值 | 模式 | 用途 |
|---:|---|---|
| 0 | OFF | 停止识别和误差发布 |
| 1 | HIGH_TARGET | 完整双圆 + 十字复核 |
| 2 | TRANSITION | 内圆外沿 + 中心十字同时验证 |
| 3 | LOW_CROSS | 低空十字交点 |

切换示例：

```bash
ros2 topic pub --once /land_air/vision_mode \
  std_msgs/msg/UInt8 "{data: 1}"
```

### 中心误差输出

```text
/fine_data    std_msgs/msg/Int32MultiArray
[dx_px, dy_px]
```

- `dx_px > 0`：目标位于画面右侧。
- `dy_px > 0`：目标位于画面下侧。
- `[0, 0]`：目标位于图像中心，不表示目标丢失。
- 目标无效时停止发布 `/fine_data`。

查看输出：

```bash
ros2 topic echo /fine_data
```

### 视觉状态输出

```text
/land_air/vision_status    std_msgs/msg/Int32MultiArray
[mode, valid, confidence_milli]
```

- `mode`：当前 0-3 模式。
- `valid`：`1` 有效，`0` 无效。
- `confidence_milli`：置信度乘以 1000，例如 `850` 表示 `0.850`。

```bash
ros2 topic echo /land_air/vision_status
```

三个话题名称均可通过参数 `mode_topic`、`fine_topic`、`status_topic` 修改。

## 关键参数

查看全部参数：

```bash
ros2 param list /land_air_vision
ros2 param dump /land_air_vision
```

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `camera.index` | 0 | `/dev/videoN` 中的 N |
| `camera.width` | 640 | 采集宽度 |
| `camera.height` | 480 | 采集高度 |
| `camera.fps` | 30 | 采集和定时器目标帧率 |
| `camera.fourcc` | MJPG | USB 摄像头压缩格式 |
| `opencv_threads` | 4 | OpenCV CPU 线程数 |
| `show_dashboard` | true | 是否显示检测画面 |
| `initial_mode` | 0 | 启动识别模式；默认 OFF，等待控制端指令 |
| `circle.min_confidence` | 0.70 | 双圆最低置信度 |
| `circle.require_cross_validation` | 1 | 高空模式是否强制十字复核 |
| `circle.diameter_ratio_tolerance` | 0.18 | 30/50 圆径比例容差 |
| `circle.enable_cross_guided_recovery` | 1 | 圆轮廓断开时启用十字中心约束的双半径恢复 |
| `circle.cross_guided_min_visible_fraction` | 0.82 | 恢复圆至少应位于画面内的圆周比例 |
| `circle.outer_edge_search_ratio` | 0.20 | 从候选半径向外搜索黑色圆环外沿的范围 |
| `circle.outer_edge_relative_support` | 0.65 | 外沿相对该圆最强边缘的最低支持率 |
| `transition.inner_min_radius_ratio` | 0.10 | Mode 2 内圆外沿相对图像短边的最小半径 |
| `transition.inner_edge_separation_expected_ratio` | 0.125 | 2 cm 线宽相对内圆外沿半径的预期比例 |
| `cross.max_segments` | 80 | 模式 2/3 参与组合的最长线段上限 |
| `cross.parallel_tolerance_deg` | 9.0 | 同一粗笔画两条边的平行角容差 |
| `cross.min_stroke_width_px` | 4.0 | 可配对的最小笔画像素宽度 |
| `cross.max_stroke_width_ratio` | 0.28 | 最大笔画宽度相对短边比例 |
| `cross.max_centerlines` | 36 | 配对后参与正交组合的中心线上限 |
| `filter.ema_alpha` | 0.38 | 中心响应速度 |
| `filter.radius_ema_alpha` | 0.40 | 圆半径响应速度 |
| `tracking.acquire_frames` | 3 | 连续确认帧数 |
| `tracking.max_missed_frames` | 3 | 短时丢失保持帧数 |
| `tracking.max_radius_change_ratio` | 0.22 | 单次半径变化门限 |
| `tracking.max_center_jump_px` | 45.0 | 小目标时中心变化门限 |

运行时调参示例：

```bash
ros2 param set /land_air_vision filter.radius_ema_alpha 0.45
ros2 param set /land_air_vision tracking.max_radius_change_ratio 0.25
ros2 param set /land_air_vision circle.min_confidence 0.72
ros2 param set /land_air_vision cross.max_segments 80
ros2 param set /land_air_vision cross.max_stroke_width_ratio 0.28
```

模式 3 帧率不足时，可将 `cross.max_segments` 从 `80` 降到 `60`；如果复杂背景下目标十字线段偶尔未进入候选集合，可提高到 `100`。该参数修改会立即重置跟踪器并生效，不需要重启节点。

运行时参数只保留到本次进程退出。需要永久保存时，可使用：

```bash
ros2 param dump /land_air_vision > vision_params.yaml
```

下次启动：

```bash
python3 vision_target.py --ros2 --ros-args \
  --params-file vision_params.yaml
```

## 上机调试顺序

1. 使用 `v4l2-ctl` 确认 USB 摄像头编号、分辨率、帧率和 MJPG 支持。
2. 启动节点并确认画面窗口能实时显示。
3. 发布模式 `1`，完整展示标靶，确认蓝色圆、黄色十字和绿色中心稳定。
4. 查看 `/land_air/vision_status`，确认连续跟踪时 `valid=1`。
5. 平移标靶，检查 `/fine_data` 的左右、上下符号。
6. 改变相机高度，检查圆半径响应和 `rejected_radius_jump` 日志。
7. 按 `1→2→3` 切换：Mode 1 应标出两个圆；Mode 2 外圆出界后仍标出内圆和十字；Mode 3 只要求十字。十字交点移出画面后应立即无效。
8. 最后再接入飞控闭环；目标无效时控制端必须停止使用旧误差。

## 与 Demo 总体框架的接口对应

本节点只承担视觉模块职责，不决定起飞、伴飞、下降、投放或任务状态：

- 接收 `/land_air/vision_mode` 的 `0/1/2/3` 分段视觉命令。
- 只在当前帧目标有效时向 `/fine_data` 发布 `[dx_px, dy_px]`。
- 无效、获取中、短时保持和丢失状态均停止发布误差，不用历史中心冒充当前结果。
- 向 `/land_air/vision_status` 发布显示和记录状态，不直接参与飞控闭环。
- 高空使用双圆并以十字复核；过渡阶段严格使用内圆加十字；低空只使用正交十字交点。
- 所有识别、滤波和跳变门限均作为 ROS2 参数集中配置，没有散落在任务状态逻辑中。

## 常见问题

### 无法打开摄像头

```bash
ls -l /dev/video*
groups
fuser /dev/video0
```

确认设备存在、用户属于 `video` 组，并且摄像头没有被其他程序占用。

### 摄像头帧率低

确认摄像头支持 MJPG：

```bash
v4l2-ctl -d /dev/video0 --list-formats-ext
```

必要时降低到 640x480，并保持 `camera.fourcc:=MJPG`。

### 无桌面环境启动失败

关闭画面窗口：

```bash
python3 vision_target.py --ros2 --ros-args -p show_dashboard:=false
```

### 能看到目标但没有 `/fine_data`

查看状态和日志：

```bash
ros2 topic echo /land_air/vision_status
```

只有 `TRACKING + valid=1` 才会发布 `/fine_data`。`ACQUIRING`、`HOLD`、`LOST` 均不会输出旧坐标。

## 安全说明

本节点只提供图像误差，不负责起飞、下降、急停或任务状态决策。首次联调应拆除桨叶或将飞控输出置于安全状态，确认模式切换、误差符号、丢失处理和延迟均正确后再进行受控飞行测试。
