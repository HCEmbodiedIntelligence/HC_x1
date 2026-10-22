#!/usr/bin/env python3
"""Move the three waist joints through Redis action 30 with safety checks."""

from __future__ import annotations

import argparse
import json
import math
import subprocess
import sys
import time

import redis


LOWER_DEG = (-36.0, -24.0, -105.0)
UPPER_DEG = (62.0, 135.0, 64.0)


def service_active(name: str) -> bool:
    return subprocess.run(
        ["systemctl", "is-active", "--quiet", name], check=False
    ).returncode == 0


def bridge_running() -> bool:
    return subprocess.run(
        ["pgrep", "-f", "/hc_tj_marvin_bridge_node( |$)"],
        check=False,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    ).returncode == 0


def read_status(client: redis.Redis) -> tuple[list[float], float]:
    raw = client.get("waist_status")
    if not raw:
        raise RuntimeError("Redis 中没有 waist_status")
    status = json.loads(raw)
    joints = [float(value) for value in status["joint"]]
    if len(joints) != 3 or not all(math.isfinite(value) for value in joints):
        raise RuntimeError(f"waist_status.joint 无效: {joints!r}")
    timestamp_ns = int(status.get("timestamp", 0))
    age_sec = (time.time_ns() - timestamp_ns) / 1e9 if timestamp_ns else math.inf
    return joints, age_sec


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--target-rad",
        type=float,
        nargs=3,
        metavar=("LEG_1", "LEG_2", "ZHI"),
        required=True,
        help="目标关节角，单位弧度",
    )
    parser.add_argument("--velocity-deg-s", type=float, default=5.0)
    parser.add_argument("--tolerance-deg", type=float, default=0.5)
    parser.add_argument("--move-timeout", type=float, default=30.0)
    parser.add_argument("--redis-host", default="127.0.0.1")
    parser.add_argument("--redis-port", type=int, default=6379)
    parser.add_argument(
        "--execute",
        action="store_true",
        help="实际发送控制命令；不加此参数只检查并显示目标",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    target_deg = [math.degrees(value) for value in args.target_rad]
    if not all(math.isfinite(value) for value in target_deg):
        raise RuntimeError("目标包含非有限数值")
    for name, value, lower, upper in zip(
        ("leg_1", "leg_2", "zhi"), target_deg, LOWER_DEG, UPPER_DEG
    ):
        if not lower <= value <= upper:
            raise RuntimeError(
                f"{name}={value:.6f}° 超出限位 [{lower:.1f}, {upper:.1f}]°"
            )
    if not 0.0 < args.velocity_deg_s <= 90.0:
        raise RuntimeError("--velocity-deg-s 必须在 (0, 90] 内")
    if not 0.05 <= args.tolerance_deg <= 5.0:
        raise RuntimeError("--tolerance-deg 必须在 [0.05, 5] 内")
    if args.move_timeout <= 0.0:
        raise RuntimeError("--move-timeout 必须大于 0")

    client = redis.Redis(
        host=args.redis_host,
        port=args.redis_port,
        decode_responses=True,
        socket_connect_timeout=0.5,
        socket_timeout=0.5,
    )
    client.ping()
    current_deg, age_sec = read_status(client)
    print(f"当前角度: {[round(value, 6) for value in current_deg]} deg")
    print(f"目标角度: {[round(value, 6) for value in target_deg]} deg")
    print(f"最大速度: {args.velocity_deg_s:.3f} deg/s")

    if not args.execute:
        print("仅检查，未发送命令；确认后加 --execute")
        return 0
    if service_active("joystick-control.service"):
        raise RuntimeError(
            "joystick-control.service 正在写 waist_move；先停止它，再单独启动 "
            "watchdog-waist-control.service"
        )
    if not service_active("watchdog-waist-control.service"):
        raise RuntimeError("watchdog-waist-control.service 未运行")
    if bridge_running():
        raise RuntimeError(
            "检测到 HC_X1 腰部桥接进程；请先停止 HC_X1，避免两个 action 30 写入源"
        )
    if age_sec > 0.5:
        raise RuntimeError(f"waist_status 已停更 {age_sec:.3f}s")

    deadline = time.monotonic() + args.move_timeout
    period_sec = 0.02
    timeout_ms = 100
    try:
        while True:
            current_deg, age_sec = read_status(client)
            if age_sec > 0.5:
                raise RuntimeError(f"运动期间 waist_status 停更 {age_sec:.3f}s")
            errors = [abs(a - b) for a, b in zip(current_deg, target_deg)]
            if max(errors) <= args.tolerance_deg:
                print(
                    "已到位: "
                    f"{[round(value, 6) for value in current_deg]} deg，"
                    f"最大误差 {max(errors):.4f}°"
                )
                return 0
            if time.monotonic() >= deadline:
                raise RuntimeError(
                    f"{args.move_timeout:.1f}s 内未到位；当前 "
                    f"{[round(value, 6) for value in current_deg]} deg"
                )
            now_ns = time.time_ns()
            command = {
                "action": 30,
                "armID": 3,
                "params": {
                    "joint": target_deg,
                    "timestamp_ns": now_ns,
                    "max_velocity_deg_s": args.velocity_deg_s,
                    "timeout_ms": timeout_ms,
                },
                "taskID": now_ns,
            }
            client.set("waist_move", json.dumps(command, separators=(",", ":")))
            time.sleep(period_sec)
    except KeyboardInterrupt:
        print("\n已停止刷新 action 30；控制器看门狗将在 100 ms 后保持当前位置")
        return 130


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (RuntimeError, redis.RedisError, json.JSONDecodeError, KeyError, ValueError) as exc:
        print(f"[redis_move_waist ERR] {exc}", file=sys.stderr)
        raise SystemExit(1)
