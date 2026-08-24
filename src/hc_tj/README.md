# HC-TJ X1 drivers

该目录包含 X1 上半身、底盘和相机 ROS 2 驱动的组合启动脚本。请从项目根目录使用 `build.sh` 和 `start.sh`，不要直接依赖其他源码工作空间。

组件：

- `hc_tj_marvin_bridge_cpp`：Marvin 双臂和腰部状态/命令合成；
- `hc_tj_body_bridge`：底盘 Redis 桥；
- `hc_tj_camera_bridge`：头部相机压缩、深度 PNG 和标定桥；
- `run_hc_tj_all.sh`：供项目根启动器调用的 X1 私有驱动组。

厂商 SDK 位于项目根 `vendor/`，相机配置位于 `config/camera/`。腰部和底盘的机器级 Redis 服务要求见 `../../docs/DEPLOYMENT.md`。
