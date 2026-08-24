# HC_X1

X1 机器人独立硬件适配项目。它启动 X1 的双臂、腰部、底盘、灵巧手和相机驱动，并向通用 `HC-teleop-robotic` 产品提供稳定的 `/hc_teleop` ROS 2 标准接口。

本项目自带所需 ROS 包和 aarch64 厂商运行库，不依赖 `hc_io_suit`、`HC-teleop-robotic` 或用户目录中的其他源码工作空间。腰部和底盘仍依赖 X1 机器上已经部署的 Redis/硬件系统服务，详见 [部署说明](docs/DEPLOYMENT.md)。

## 目录

```text
HC_X1/
├── hc_x1/                 # 标准话题适配与 100 Hz 关节命令执行器
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

通用系统独立启动：

```bash
cd ~/HC-teleop-robotic
ROS_DOMAIN_ID=14 ./start_real_robot_teleop.sh
```

两个项目可以部署在同机或不同机器，但 `ROS_DOMAIN_ID` 必须一致，跨机时 `ROS_LOCALHOST_ONLY=0`。

## 控制链路

```text
HC Teleop /hc_teleop/joint_cmd
  -> hc_x1_joint_command_smoother（100 Hz、限速/限加速度/超时）
  -> /io_teleop/joint_cmd
  -> Marvin X1 driver

X1 drivers /io_teleop/*
  -> hc_x1_standard_adapter
  -> HC Teleop /hc_teleop/*
```

硬件私有话题不会泄漏给通用系统。相机桥只发布标准压缩流供网页和 MCAP 录制，避免录制重复别名和不必要的未压缩图像。
