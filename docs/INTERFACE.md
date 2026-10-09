# X1 标准接口映射

## 控制输入

| 标准输入 | X1 私有输出 | 类型 |
| --- | --- | --- |
| `/action/joint_states` | `/io_teleop/joint_cmd` | `sensor_msgs/msg/JointState` |
| `/action/base/command` | `/io_teleop/target_base_move` | `std_msgs/msg/Float64MultiArray` |
| `/action/left_hand/joint_states` | `/io_teleop/joint_cmd_finger_left` | `sensor_msgs/msg/JointState` |
| `/action/right_hand/joint_states` | `/io_teleop/joint_cmd_finger_right` | `sensor_msgs/msg/JointState` |

关节命令是专用平滑节点唯一拥有的 X1 私有输出。控制适配器不会转发机械臂关节命令；它与相机适配器分属独立进程，图像回调不会阻塞关节反馈、手指或底盘。关节反馈使用 Reliable + `KEEP_LAST(1)`，兼容 IK 订阅者且不积压旧帧。

## 状态输出

| X1 私有输入 | 标准输出 | 类型 |
| --- | --- | --- |
| `/io_teleop/joint_states` | `/observation/joint_states` | `sensor_msgs/msg/JointState` |
| `/io_teleop/hand_joint_states` | `/observation/hands/joint_states` | `sensor_msgs/msg/JointState` |
| `/io_teleop/hand_trigger` | `/observation/hands/triggers` | `geometry_msgs/msg/PoseArray` |
| `/io_teleop/chassis_state` | `/status/base` | `std_msgs/msg/String` |
| `/io_teleop/camera_head/color[/compressed]` | `/observation/cameras/head/color/compressed` | `sensor_msgs/msg/CompressedImage` |
| `/io_teleop/camera_head/depth` | `/observation/cameras/head/depth/compressed` | `sensor_msgs/msg/CompressedImage` |
| `/io_teleop/camera_d405_{left,right}/color/compressed` | `/observation/cameras/d405_{left,right}/color/compressed` | `sensor_msgs/msg/CompressedImage` |
| `/io_teleop/camera_d405_{left,right}/depth/compressed` | `/observation/cameras/d405_{left,right}/depth/compressed` | `sensor_msgs/msg/CompressedImage` |

相机适配器只转发 `CompressedImage`，不订阅 raw `sensor_msgs/Image`，也不在 Python 中进行 JPEG 编码。OmniHand 使用上表中的标准左右手指命令，不再保留 `/vrdata` 到 X1 私有消息的旁路。
