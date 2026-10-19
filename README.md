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
