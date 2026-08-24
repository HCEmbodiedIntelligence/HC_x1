# X1 标准接口映射

## 控制输入

| 标准输入 | X1 私有输出 | 类型 |
| --- | --- | --- |
| `/hc_teleop/joint_cmd` | `/io_teleop/joint_cmd` | `sensor_msgs/msg/JointState` |
| `/hc_teleop/target_base_move` | `/io_teleop/target_base_move` | `std_msgs/msg/Float64MultiArray` |
| `/hc_teleop/joint_cmd_finger_left` | `/io_teleop/joint_cmd_finger_left` | `sensor_msgs/msg/JointState` |
| `/hc_teleop/joint_cmd_finger_right` | `/io_teleop/joint_cmd_finger_right` | `sensor_msgs/msg/JointState` |

关节命令是专用平滑节点唯一拥有的 X1 私有输出。标准适配节点不会转发机械臂关节命令，因此相机或状态回调阻塞不会改变伺服节拍。

## 状态输出

| X1 私有输入 | 标准输出 | 类型 |
| --- | --- | --- |
| `/io_teleop/joint_states` | `/hc_teleop/joint_states` | `sensor_msgs/msg/JointState` |
| `/io_teleop/hand_joint_states` | `/hc_teleop/hand_joint_states` | `sensor_msgs/msg/JointState` |
| `/io_teleop/hand_trigger` | `/hc_teleop/hand_trigger` | `geometry_msgs/msg/PoseArray` |
| `/io_teleop/chassis_state` | `/hc_teleop/chassis_state` | `std_msgs/msg/String` |
| `/io_teleop/camera_head/color[/compressed]` | `/hc_teleop/camera_head/color/compressed` | `sensor_msgs/msg/CompressedImage` |
| `/io_teleop/camera_head/depth` | `/hc_teleop/camera_head/depth/compressed` | `sensor_msgs/msg/CompressedImage` |
| `/io_teleop/camera_d405_{left,right}/color/compressed` | `/hc_teleop/camera_d405_{left,right}/color/compressed` | `sensor_msgs/msg/CompressedImage` |
| `/io_teleop/camera_d405_{left,right}/depth/compressed` | `/hc_teleop/camera_d405_{left,right}/depth/compressed` | `sensor_msgs/msg/CompressedImage` |

头部和头顶相机支持原驱动 raw 图像的 JPEG 回退，但检测到已有压缩流时不会重复压缩发布。

## VR 兼容输入

X1 OmniHand 驱动仍需要私有 `io_msgs2/msg/VrData`。`hc_x1_standard_adapter` 将通用 `/vrdata` JSON 转换为该消息，同时兼容 `primary_axis`/旧 `primary` 和 `quaternion`/旧 `orientation` 字段。
