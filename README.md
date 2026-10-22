# HC_X1

X1 机器人独立硬件适配项目。它启动 X1 的双臂、腰部、底盘、灵巧手和相机驱动，并向通用 `HC-teleop-robotic` 产品提供稳定的 `/action、/observation、/control、/status` ROS 2 标准接口。

本项目自带所需 ROS 包和 aarch64 厂商运行库，不依赖 `hc_io_suit`、`HC-teleop-robotic` 或用户目录中的其他源码工作空间。腰部和底盘仍依赖 X1 机器上已经部署的 Redis/硬件系统服务，详见 [部署说明](docs/DEPLOYMENT.md)。

## 目录

```text
HC_X1/
├── hc_x1/                 # 独立控制/相机适配与 100 Hz 关节命令执行器
├── config/                # X1 硬件侧参数
├── src/                   # ROS 2 驱动与消息包
├── vendor/                # Marvin、OmniHand、相机 aarch64 SDK
├── docs/                  # 接口和部署文档
├── build.sh
├── start.sh
└── check.sh
```

## 构建与启动

```bash
cd ~/HC_X1
./build.sh
ROS_DOMAIN_ID=14 ./start.sh
```

未安装 D405 时使用 `./start.sh --no-d405`。其他可选项包括 `--no-camera`、`--no-head-camera`、`--no-chassis` 和 `--no-hands`。启动后可另开终端执行 `ROS_DOMAIN_ID=14 ./check.sh`。

### 设备与控制开关

设备开关会完全不启动相应硬件接口；控制开关会保留状态反馈，但禁止下发命令、
启动回零和手动复位回零。机械臂只读时也不会切换该侧控制模式。

| 参数 | 效果 |
| --- | --- |
| `arms:=false` | 不连接 Marvin 双臂，仅保留其他已启用设备 |
| `arms_control:=false` | 双臂只读，保留 14 关节反馈 |
| `left_arm_control:=false` | 左臂只读，右臂仍可控制 |
| `right_arm_control:=false` | 右臂只读，左臂仍可控制 |
| `waist:=false` | 不连接腰部 Redis，不发布腰部关节 |
| `waist_control:=false` | 腰部只读，不写 `waist_move` action 30 |
| `chassis:=false` | 不启动底盘桥接 |
| `chassis_control:=false` | 底盘只读，只发布 `agvState`，不写 `taskGroupCmd` |
| `hands:=false` | 不启动 OmniHand 灵巧手桥接 |
| `camera:=false` | 不启动任何相机 |
| `head_camera:=false` | 不启动头部相机 |
| `d405:=false` | 不启动双 D405 |

例如仅允许右臂运动，左臂、腰部和底盘保持只读：

```bash
ROS_DOMAIN_ID=14 ./start.sh \
  left_arm_control:=false \
  waist_control:=false \
  chassis_control:=false
```

保留全部关节和底盘反馈、禁止双臂/腰部/底盘控制：

```bash
ROS_DOMAIN_ID=14 ./start.sh \
  arms_control:=false \
  waist_control:=false \
  chassis_control:=false
```

可随时运行 `./start.sh --help` 查看支持的参数，即使另一个 HC_X1 实例正在运行。

`start.sh` 会在前台等待双臂初始化、启动回零（启用时）和标准关节接口就绪，
通过后才打印“HC_X1 已启动”。若控制模式验证失败，会直接打印原始
`A/B(ERR, State, ImpType)` 错误、详细日志路径，以非零状态退出并清理本次启动的进程。
即使驱动节点仍存活，也不会把双臂连接失败视为启动成功。
等待上限默认 60 秒，可通过 `HC_X1_STARTUP_TIMEOUT_SEC` 调整（最大 600 秒）。
`arms:=false`、`waist:=false` 和配置中的腰部禁用会相应调整反馈检查。
此检查只读取日志和 ROS 状态，不执行复位或附加运动命令；原有启动自动回零配置仍然生效。

通用系统独立启动：

```bash
cd ~/HC-teleop-robotic
ROS_DOMAIN_ID=14 ./run.sh teleop --config middleware/config.x1_jetson.yaml
```

两个项目可以部署在同机或不同机器，但 `ROS_DOMAIN_ID` 必须一致，跨机时 `ROS_LOCALHOST_ONLY=0`。

## 控制链路

```text
HC Teleop /action/joint_states
  -> hc_x1_joint_command_smoother（100 Hz、限速/限加速度/超时）
  -> /io_teleop/joint_cmd
  -> Marvin X1 driver

X1 control/state /io_teleop/*
  -> hc_x1_control_adapter（独立进程、KEEP_LAST(1)）
  -> HC Teleop control/state /action/*、/observation/*、/control/*、/status/*

X1 compressed cameras /io_teleop/camera_*
  -> hc_x1_camera_adapter（独立进程）
  -> HC Teleop compressed cameras /observation/cameras/*
```

硬件私有话题不会泄漏给通用系统。相机适配器只订阅已经压缩的图像，
不再在 Python 中订阅或编码 raw 图像；相机负载不会阻塞关节、手指和底盘回调。

## Redis 腰部定点工具

`tools/redis_move_waist.py` 使用腰部控制器的 action 30，以 50 Hz 刷新唯一
`taskID`，按反馈判断到位；停止刷新后由控制器看门狗保持当前位置。工具接收弧度
目标，发送前检查关节限位、反馈时效和冲突写入源。

直接控制前必须停止 HC_X1，并停止持续写 `waist_move` 的北通手柄服务。由于停止
`joystick-control.service` 会连带停止腰部驱动，需要再单独启动腰部驱动：

```bash
sudo systemctl stop joystick-control.service
sudo systemctl start watchdog-waist-control.service
sleep 2
```

先检查目标和当前反馈，不发送命令：

```bash
./tools/redis_move_waist.py --target-rad LEG_1 LEG_2 ZHI
```

确认实体急停可触达后，低速执行：

```bash
./tools/redis_move_waist.py \
  --target-rad LEG_1 LEG_2 ZHI \
  --velocity-deg-s 5 \
  --execute
```

按 `Ctrl+C` 会停止刷新 action 30，控制器在 100 ms 超时后保持当前位置。
