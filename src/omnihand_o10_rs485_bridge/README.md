# OmniHand O10 RS485 桥接

基于 `~/examples/main_rs485_ros2.cc` 重构，使用 **AGILINK OmniHand2025 SDK**（`createHandByRs485`）对接 io_teleop 外骨骼手指指令，并保留触觉震动反馈。

## 硬件

- O10 灵巧手 × 2，左右各一条 RS485 总线
- 左手：`/dev/ttyTHS2`
- 右手：`/dev/ttyTHS0`
- 波特率 **460800**，手须 **24V 供电**

## 依赖

AGILINK OmniHand SDK（默认路径）：

```text
~/hc_human/agillink_omnihand_sdk/linux/aarch64/cpp
```

编译时可指定：

```bash
colcon build --packages-select omnihand_o10_rs485_bridge \
  --cmake-args -DOMNIHAND_SDK_ROOT=/path/to/agillink_omnihand_sdk/linux/aarch64/cpp
```

## 编译

```bash
source /opt/ros/humble/setup.bash
cd ~/io_teleop_v3_ros2_ws
colcon build --packages-select omnihand_o10_rs485_bridge
source install/setup.bash
```

## 启动

```bash
./src/omnihand_o10_rs485_bridge/run_omnihand_rs485.sh
```

启动脚本会先执行以下 Jetson UART 引脚复用配置，再检查
`/dev/ttyTHS0`、`/dev/ttyTHS2` 并启动 ROS 2 节点：

```bash
sudo busybox devmem 0x0243d070 w 0x00000400
sudo busybox devmem 0x0243d078 w 0x00000458
```

使用 `run_hc_tj_all.sh` 一键启动时，这一步会在前台执行，以便正常输入 sudo
密码；初始化失败时不会继续启动机器人。

或指定参数：

```bash
ros2 launch omnihand_o10_rs485_bridge omnihand_o10_rs485_bridge.launch.py \
  left_uart_port:=/dev/ttyTHS2 right_uart_port:=/dev/ttyTHS0
```

## ROS2 接口

| 类型 | 话题 | 说明 |
|------|------|------|
| 订阅 | `/action/left_hand/joint_states` | 左手 10 关节 rad |
| 订阅 | `/action/right_hand/joint_states` | 右手 10 关节 rad |
| 订阅 | `/control/vr/io_data` | 可选扳机模式：左右中指（握把）扳机分别控制左右手 |
| 发布 | `/observation/hands/vibration_feedback` | 外骨骼震动反馈 `Float64MultiArray[10]`（0–4 右手五指，5–9 左手五指，范围 0–10） |
| 发布 | `/observation/hands/joint_states` | `JointState`，实际反馈换算的 20 个主动关节角（右 10 + 左 10，rad） |
| 发布 | `/observation/hands/triggers` | 仅扳机直控模式；`PoseArray`，`poses[0]=左`、`poses[1]=右` |

外骨骼关节 rad 经映射表线性插值到电机位置 0–4096（对齐 `main_rs485_ros2.cc`），**100 Hz（10ms）** 周期下发 `SetAllJointMotorPosi`。触觉通过 `GetTactileSensorData` 读取，默认 **10 Hz** 发布到 `/observation/hands/vibration_feedback`（**BEST_EFFORT**）。

## 参数

| 参数 | 默认 | 说明 |
|------|------|------|
| `left_uart_port` | `/dev/ttyTHS2` | 左手串口 |
| `right_uart_port` | `/dev/ttyTHS0` | 右手串口 |
| `uart_baudrate` | `460800` | 波特率 |
| `hand_device_id` | `1` | RS485 设备 ID |
| `enable_vibration_feedback` | `true` | 是否发布外骨骼震动反馈 |
| `vibration_topic` | `/observation/hands/vibration_feedback` | 震动反馈话题 |
| `vibration_rate_hz` | `10.0` | 触觉发布频率（Hz） |
| `hand_joint_state_topic` | `/observation/hands/joint_states` | 实际手指关节状态话题 |
| `hand_joint_state_rate_hz` | `20.0` | 实际手指关节状态发布频率（Hz） |
| `hand_trigger_topic` | `/observation/hands/triggers` | 标准 `geometry_msgs/msg/PoseArray` 中指扳机话题 |
| `ignore_placeholder_finger_cmd` | `true` | 忽略 TeleXperience 全零占位指令 |
| `enable_vr_trigger_control` | `false` | 启用 VR 中指（握把）扳机线性控制；启用后屏蔽标准手指命令 |
| `vr_data_topic` | `/control/vr/io_data` | `io_msgs2/msg/VrData` 输入话题，仅启用扳机直控时订阅 |
| `vr_close_time_sec` | `0.6` | 从张开到握拳的最短线性过渡时间 |
| `vr_open_time_sec` | `0.6` | 从握拳到张开的最短线性过渡时间 |
| `vr_command_timeout_sec` | `0.5` | VR 数据超时后自动张开的等待时间 |

扳机模式优先使用中指（握把）`l_hand_axis/r_hand_axis` 的 0–1 模拟量，数字
`l_hand_trigger/r_hand_trigger` 用作只提供按键状态的设备兜底。电机位置在以下两端间
逐轴插值：

```text
张开: [3800, 4096, 4096, 1899, 4096, 4096, 4096, 4096, 0, 4096]
握拳: [3800, 4096,   66, 1899, 2300, 2300, 1932,  604, 2094,  408]
```

## 参考

- 遥操作样例：`~/examples/main_rs485_ros2.cc`
- SDK：`~/hc_human/agillink_omnihand_sdk`
