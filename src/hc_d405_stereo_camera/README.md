# hc_d405_stereo_camera

直接使用 niic 已安装的 librealsense，同时驱动两台 Intel RealSense D405。无需安装
`realsense2_camera` ROS 包。

## 实机映射

- left：`260322275504`
- right：`260322276941`

序列号与物理左右可在 `config/d405_stereo.yaml` 中交换。

每台相机发布：

| 后缀 | 类型 | 内容 |
|---|---|---|
| `color/image_raw` | `sensor_msgs/Image` | `bgr8` 彩色原图 |
| `color/compressed` | `sensor_msgs/CompressedImage` | JPEG 彩色图 |
| `color/camera_info` | `sensor_msgs/CameraInfo` | 彩色内参/畸变 |
| `depth/image_raw` | `sensor_msgs/Image` | `16UC1` 深度原图 |
| `depth/compressed` | `sensor_msgs/CompressedImage` | 无损 16 位 PNG |
| `depth/camera_info` | `sensor_msgs/CameraInfo` | 深度内参/畸变 |
| `depth/scale` | `std_msgs/Float32` | 每个 `uint16` 深度单位对应的米数（实测为 `0.0001`） |
| `info` | `std_msgs/String` | 2 Hz 版本化 JSON，合并彩色/深度内参与 depth scale，供 TeleBox 录制 |

左右命名空间分别为：

- `/io_teleop/camera_d405_left`
- `/io_teleop/camera_d405_right`

图像话题使用 `RELIABLE + VOLATILE + KEEP_LAST(5)`，`info` 使用
`RELIABLE + TRANSIENT_LOCAL + KEEP_LAST(1)`。TeleBox 分别将原头部、
D405 left、D405 right 配置为 `video_0/video_1/video_2`，所以三路画面可同时进入 VR：

- `video_0`: `/io_teleop/camera_head/color`
- `video_1`: `/io_teleop/camera_d405_left/color/compressed`
- `video_2`: `/io_teleop/camera_d405_right/color/compressed`

D405 包默认不发布 `/io_teleop/camera_head/color`，可与原 `hc_tj_camera_bridge` 同时运行。
可选的左右拼接流默认关闭；若开启则使用独立的
`/io_teleop/camera_d405/vr/composite`。
TeleBox 配置片段见 `config/telebox_data_proxy_videos.yaml`。

左右 D405 是可选硬件，由 `config/camera_presence.env` 配置：

```bash
HC_D405_LEFT_ENABLED=true
HC_D405_RIGHT_ENABLED=true
```

设为 `false` 的相机不会打开设备、创建发布者或尝试重连，也不会影响另一台相机和全栈启动。
配置为 `true` 但设备不存在时才作为硬件配置错误处理。一次性覆盖可使用
`run_hc_tj_all.sh --no-d405-left` 或 `--no-d405-right`；修改持久配置后需重启全栈。

当前 TeleBox 录制配置对每台 D405 只保留 `color/compressed`、
`depth/compressed` 和 `info`，不再同时录制 raw 与 compressed。

TeleBox schema 3.0 不允许在自定义映射中使用 `sensor_msgs/CameraInfo` 和
`std_msgs/Float32`。这些 ROS 话题仍正常发布；TeleBox 通过周期发布的
`std_msgs/String` `info` 标定 JSON 录制内参和深度比例。

## 构建和运行

```bash
cd ~/HC_X1
source /opt/ros/humble/setup.bash
colcon build --packages-select hc_d405_stereo_camera
source install/setup.bash
./src/hc_d405_stereo_camera/run_camera.sh
```

完整机器人推荐直接运行 `./src/hc_tj/run_hc_tj_all.sh`，它会同时启动原头部相机和双 D405。
不要再并行运行本节的独立 D405 命令；同一相机被第二个进程打开时会报
`VIDIOC_S_FMT ... errno=16 (Device or resource busy)`。
启动脚本通过 `/tmp/hc_d405_stereo_camera.lock` 和进程检查拒绝重复实例。

临时交换左右画面：

```bash
./src/hc_d405_stereo_camera/run_camera.sh \
  left_serial:=260322276941 right_serial:=260322275504
```

临时修改可选合成图话题：

```bash
./src/hc_d405_stereo_camera/run_camera.sh \
  vr_topic:=/io_teleop/camera_d405/vr/composite
```

## 验证

```bash
ros2 topic hz /io_teleop/camera_d405_left/color/compressed
ros2 topic hz /io_teleop/camera_d405_right/color/compressed
ros2 topic echo /io_teleop/camera_d405_left/color/camera_info --once
ros2 topic echo /io_teleop/camera_d405_right/depth/camera_info --once
ros2 topic echo /io_teleop/camera_d405_left/depth/scale --once
ros2 topic hz /io_teleop/camera_head/color
ros2 topic info --verbose /io_teleop/camera_d405_left/color/compressed
```
