#!/usr/bin/env python3
"""Read-only startup checks for the X1 launcher; never publish robot commands."""
from __future__ import annotations

import argparse
import math
import os
from pathlib import Path
import re
import sys
import time

FAILURES = (
    '控制模式验证失败', '控制模式反馈验证失败', '控制模式指令提交失败',
    'Marvin 机器人连接失败', '无法连接到 Marvin 控制器',
    'Marvin UDP 数据通道未就绪', '运动学初始化失败', '无法加载运动学配置',
    '运动学参数初始化失败', '腰部回零超时',
)


def inspect_log(text):
    """Return original error lines and whether the whole startup home completed."""
    lines = text.splitlines()
    errors = [line for line in lines if any(value in line for value in FAILURES)]
    completed = any(re.search(r'(?:^|:\s*)回零完成(?:（[^\n]*）)?\s*$', line) for line in lines)
    return errors, completed


def expected_joints(config, launch_args):
    enabled = {'arms': True, 'waist': True}
    for arg in launch_args:
        name, sep, value = arg.partition(':=')
        if sep and name in enabled:
            if value.lower() not in {'true', 'false', '1', '0', 'yes', 'no', 'on', 'off'}:
                raise ValueError(f'invalid launch argument: {arg}')
            enabled[name] = value.lower() in {'true', '1', 'yes', 'on'}
    joints = []
    if enabled['arms']:
        for side in ('left_arm', 'right_arm'):
            names = config.get(side, {}).get('arm_joints', [])
            if len(names) != 7:
                raise ValueError(f'{side}.arm_joints must contain seven joints')
            joints.extend(names)
    if enabled['waist'] and config.get('waist', {}).get('enabled', True):
        joints.extend(config.get('waist', {}).get('joints', []))
    if len(set(joints)) != len(joints):
        raise ValueError('configured joint names are not unique')
    return set(joints)


def controls_motion(config, launch_args):
    """Return whether startup is allowed to move any arm or waist joint."""
    enabled = {
        'arms': True,
        'arms_control': True,
        'left_arm_control': True,
        'right_arm_control': True,
        'waist': True,
        'waist_control': True,
    }
    for arg in launch_args:
        name, sep, value = arg.partition(':=')
        if sep and name in enabled:
            if value.lower() not in {'true', 'false', '1', '0', 'yes', 'no', 'on', 'off'}:
                raise ValueError(f'invalid launch argument: {arg}')
            enabled[name] = value.lower() in {'true', '1', 'yes', 'on'}
    arms_control = (enabled['arms'] and enabled['arms_control'] and
                    (enabled['left_arm_control'] or enabled['right_arm_control']))
    waist_control = (enabled['waist'] and enabled['waist_control'] and
                     config.get('waist', {}).get('enabled', True) and
                     bool(config.get('waist', {}).get('joints', [])))
    return arms_control or waist_control


class Readiness:
    def __init__(self, expected):
        self.expected = expected
        self.joints_at = self.ready_at = float('-inf')
        self.ready = False

    def joint_message(self, message, now):
        names, positions = message.name, message.position
        valid = (len(names) == len(positions) and len(names) == len(set(names))
                 and self.expected <= set(names) and all(math.isfinite(v) for v in positions))
        self.joints_at = now if valid else float('-inf')

    def ready_message(self, message, now):
        self.ready, self.ready_at = bool(message.data), now

    def available(self, now):
        return self.ready and now - self.joints_at < 1.0 and now - self.ready_at < 1.0


def alive(pid):
    try:
        os.kill(pid, 0)
        state = Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()[0]
        return state != 'Z'
    except (ProcessLookupError, FileNotFoundError):
        return False


def wait_for_startup(log_path, pids, readiness, spin, timeout, require_home,
                     clock=time.monotonic, output=print):
    """Wait for two seconds of ready feedback, or print an actionable failure."""
    deadline, stable_since = clock() + timeout, None
    while True:
        try:
            text = log_path.read_text(errors='replace')
        except FileNotFoundError:
            text = ''
        errors, homed = inspect_log(text)
        if errors:
            output('[HC_X1 ERR] 硬件初始化失败，HC_X1 启动未成功：')
            for line in errors[-8:]:
                output(line)
            output(f'[HC_X1 ERR] 详细日志: {log_path}')
            return 1
        dead = [pid for pid in pids if not alive(pid)]
        if dead:
            output(f'[HC_X1 ERR] 启动子进程已退出: {dead}；日志目录: {log_path.parent.parent}')
            for line in text.splitlines()[-12:]:
                output(line)
            return 1
        now = clock()
        ready = (not readiness.expected or readiness.available(now)) and (not require_home or homed)
        if ready:
            if stable_since is None:
                stable_since = now
        else:
            stable_since = None
        if stable_since is not None and now - stable_since >= 2.0:
            output('[HC_X1] 初始化与标准关节接口启动检查通过。' if readiness.expected
                   else '[HC_X1] 双臂与腰部均已禁用；子进程启动检查通过。')
            return 0
        if now >= deadline:
            missing = []
            if readiness.expected and now - readiness.joints_at >= 1.0:
                missing.append('/observation/joint_states 有效完整反馈')
            if readiness.expected and (not readiness.ready or now - readiness.ready_at >= 1.0):
                missing.append('/status/hardware/ready=true')
            if require_home and not homed:
                missing.append('启动回零完成')
            output(f'[HC_X1 ERR] 启动等待超时（{timeout:g}s），尚未满足: {"、".join(missing) or "连续就绪稳定窗口"}')
            for line in text.splitlines()[-12:]:
                output(line)
            output(f'[HC_X1 ERR] 详细日志: {log_path}')
            return 1
        spin(0.1)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--log', type=Path, required=True)
    parser.add_argument('--pid', type=int, action='append', default=[])
    parser.add_argument('--timeout', type=float, default=60.)
    parser.add_argument('--launch-arg', action='append', default=[])
    args = parser.parse_args(argv)
    if not math.isfinite(args.timeout) or not 0 < args.timeout <= 600:
        parser.error('--timeout must be between 0 and 600 seconds')
    # Surface an existing failure without needing ROS discovery or SDK access.
    try:
        errors, _ = inspect_log(args.log.read_text(errors='replace'))
    except FileNotFoundError:
        errors = []
    if errors:
        return wait_for_startup(args.log, args.pid, Readiness(set()), time.sleep, args.timeout, False)
    import yaml
    import rclpy
    from ament_index_python.packages import get_package_share_directory
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Bool
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    config_path = Path(get_package_share_directory('hc_tj_marvin_bridge_cpp')) / 'config/hardware.yaml'
    config = yaml.safe_load(config_path.read_text())
    readiness = Readiness(expected_joints(config, args.launch_arg))
    require_home = bool(config.get('auto_home_on_start', True) and
                        controls_motion(config, args.launch_arg))
    rclpy.init(args=[])
    node = rclpy.create_node('hc_x1_startup_check')
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    node.create_subscription(JointState, '/observation/joint_states',
                             lambda msg: readiness.joint_message(msg, time.monotonic()), qos)
    node.create_subscription(Bool, '/status/hardware/ready',
                             lambda msg: readiness.ready_message(msg, time.monotonic()), qos)
    try:
        return wait_for_startup(args.log, args.pid, readiness,
                                lambda duration: rclpy.spin_once(node, timeout_sec=duration),
                                args.timeout, require_home)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit(130)
    except Exception as exc:
        print(f'[HC_X1 ERR] 启动检查无法完成: {exc}', file=sys.stderr)
        raise SystemExit(1)
