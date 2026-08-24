# X1 部署说明

## 平台与依赖

- Ubuntu 22.04、aarch64 Jetson、ROS 2 Humble。
- `colcon`、`yaml-cpp`、`nlohmann-json`、`hiredis`、`redis-plus-plus`、OpenCV、RealSense ROS 依赖。
- `busybox` 和有权限访问 `/dev/ttyTHS0`、`/dev/ttyTHS2` 的 `dialout` 用户。
- 厂商 SDK 已放在 `vendor/`；不要再设置指向其他源码仓库的路径。

构建前可用 rosdep 安装 ROS 依赖：

```bash
cd ~/HC_X1
sudo rosdep init 2>/dev/null || true
rosdep update
rosdep install --from-paths src --ignore-src -r -y
./build.sh
```

## 机器级硬件服务

源码已独立，但 X1 腰部和底盘的真实控制架构使用 Redis。启动前确认本机腰部 Redis 服务和底盘控制器可达：

```bash
systemctl is-active watchdog-waist-control.service
redis-cli -h 127.0.0.1 -p 6379 ping
redis-cli -h 192.168.0.3 ping
```

这些是 X1 固件/系统镜像的一部分，不属于通用中间件。未接腰部时应使用仅双臂启动参数或在硬件配置中禁用腰部，不能伪造反馈。

## 串口与相机

将运行用户加入 `dialout` 后重新登录。`start.sh` 会用 `sudo busybox devmem` 配置 Jetson UART pinmux，不包含任何硬编码密码。生产机建议配置只允许这两条寄存器操作的 sudoers 规则。

相机 udev 规则来自相应厂商 SDK。未安装 D405 时使用 `--no-d405`；不要让 systemd 相机 watchdog 与本项目重复占用设备。

## 首次验收

先在不连接通用控制项目时启动 `HC_X1`，确认关节状态和相机状态稳定。然后启动通用项目，保持实体急停可触达，从低速、小范围开始验证。若 `/io_teleop/joint_cmd` 存在第二个活动发布者，平滑节点会封锁输出；必须停止冲突控制器，不能绕过检查。
