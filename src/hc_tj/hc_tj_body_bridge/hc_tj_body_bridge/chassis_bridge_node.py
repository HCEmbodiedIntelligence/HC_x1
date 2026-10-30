#!/usr/bin/env python3
"""Bridge the standard base command to Redis and publish base status."""

import json
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String

import redis


class ChassisBridgeNode(Node):
    """Convert TeleXperience base deltas to low-speed chassis velocity commands."""

    def __init__(self) -> None:
        super().__init__("hc_tj_chassis_bridge")

        self.declare_parameter("redis_host", "192.168.0.3")
        self.declare_parameter("redis_port", 6379)
        self.declare_parameter("redis_key_command", "taskGroupCmd")
        self.declare_parameter("redis_key_state", "agvState")
        self.declare_parameter("chassis_state_topic", "/status/base")
        self.declare_parameter("base_command_topic", "/action/base/command")
        self.declare_parameter("state_poll_interval_sec", 0.1)
        self.declare_parameter("command_interval_sec", 0.08)
        self.declare_parameter("command_timeout_sec", 0.25)
        self.declare_parameter("max_vx", 0.1)
        self.declare_parameter("max_vy", 0.1)
        self.declare_parameter("max_vw", 0.2)
        self.declare_parameter("linear_gain", 1.0)
        self.declare_parameter("angular_gain", 1.0)
        self.declare_parameter("lateral_gain", 1.0)
        self.declare_parameter("stop_repeat_count", 8)
        self.declare_parameter("enable_control", True)

        host = self.get_parameter("redis_host").get_parameter_value().string_value
        port = self.get_parameter("redis_port").get_parameter_value().integer_value
        self.redis_key_command = (
            self.get_parameter("redis_key_command").get_parameter_value().string_value
        )
        self.redis_key_state = (
            self.get_parameter("redis_key_state").get_parameter_value().string_value
        )
        self.chassis_state_topic = (
            self.get_parameter("chassis_state_topic")
            .get_parameter_value()
            .string_value
        )
        self.base_command_topic = (
            self.get_parameter("base_command_topic").get_parameter_value().string_value
        )
        self.state_poll_interval_sec = (
            self.get_parameter("state_poll_interval_sec")
            .get_parameter_value()
            .double_value
        )
        self.command_interval_sec = (
            self.get_parameter("command_interval_sec")
            .get_parameter_value()
            .double_value
        )
        self.command_timeout_sec = (
            self.get_parameter("command_timeout_sec")
            .get_parameter_value()
            .double_value
        )
        self.max_vx = self.get_parameter("max_vx").get_parameter_value().double_value
        self.max_vy = self.get_parameter("max_vy").get_parameter_value().double_value
        self.max_vw = self.get_parameter("max_vw").get_parameter_value().double_value
        self.linear_gain = (
            self.get_parameter("linear_gain").get_parameter_value().double_value
        )
        self.angular_gain = (
            self.get_parameter("angular_gain").get_parameter_value().double_value
        )
        self.lateral_gain = (
            self.get_parameter("lateral_gain").get_parameter_value().double_value
        )
        self.stop_repeat_count = (
            self.get_parameter("stop_repeat_count").get_parameter_value().integer_value
        )
        self.control_enabled = (
            self.get_parameter("enable_control").get_parameter_value().bool_value
        )

        self.redis_client = redis.Redis(
            host=host,
            port=port,
            decode_responses=True,
            socket_connect_timeout=2,
            socket_timeout=2,
        )
        # State polling uses its own connection and thread so a slow GET cannot
        # delay the chassis command timer.
        self.state_redis_client = redis.Redis(
            host=host,
            port=port,
            decode_responses=True,
            socket_connect_timeout=0.2,
            socket_timeout=0.2,
        )

        self.latest_delta_yaw = 0.0
        self.latest_delta_pos = 0.0
        self.latest_delta_lateral = 0.0
        self.last_base_move_time = 0.0
        self.task_id = int(time.time() * 1000) % 2_000_000_000
        self.shutting_down = False
        self.state_stop_event = threading.Event()
        self.last_state_error_log_time = 0.0

        self.chassis_state_pub = self.create_publisher(
            String, self.chassis_state_topic, 10
        )
        if self.control_enabled:
            self.base_move_sub = self.create_subscription(
                Float64MultiArray,
                self.base_command_topic,
                self.target_base_move_callback,
                10,
            )
            self.command_timer = self.create_timer(
                self.command_interval_sec, self.command_timer_callback
            )
            self.get_logger().info(
                f"底盘控制开启：旋转/前后/横移 <- {self.base_command_topic}"
            )
        else:
            self.get_logger().info(
                "底盘只读模式：仅读取 agvState，不订阅运动目标、不写 taskGroupCmd"
            )
        self.get_logger().info(
            f"状态话题: {self.chassis_state_topic} (agvState JSON)"
        )

        try:
            self.redis_client.ping()
            state = self.redis_client.get(self.redis_key_state)
            self.get_logger().info(f"已连接底盘 Redis {host}:{port}")
            if state:
                self.get_logger().info(f"agvState: {state}")
        except redis.RedisError as exc:
            self.get_logger().error(f"底盘 Redis 连接失败: {exc}")

        self.state_thread = threading.Thread(
            target=self.state_poll_loop,
            name="chassis_state_poll",
            daemon=True,
        )
        self.state_thread.start()

    def destroy_node(self) -> bool:
        self.shutting_down = True
        self.state_stop_event.set()
        self.state_thread.join(timeout=1.0)
        if self.control_enabled:
            self.send_stop()
        return super().destroy_node()

    def state_poll_loop(self) -> None:
        while not self.state_stop_event.is_set():
            try:
                state = self.state_redis_client.get(self.redis_key_state)
                if state is not None:
                    self.chassis_state_pub.publish(String(data=state))
            except redis.RedisError as exc:
                now = time.monotonic()
                if now - self.last_state_error_log_time >= 5.0:
                    self.get_logger().error(f"读取底盘状态失败: {exc}")
                    self.last_state_error_log_time = now
            self.state_stop_event.wait(self.state_poll_interval_sec)

    def target_base_move_callback(self, msg: Float64MultiArray) -> None:
        if len(msg.data) < 2:
            return
        self.latest_delta_yaw = float(msg.data[0])
        self.latest_delta_pos = float(msg.data[1])
        self.latest_delta_lateral = float(msg.data[2]) if len(msg.data) >= 3 else 0.0
        self.last_base_move_time = time.monotonic()

    def clip(self, value: float, limit: float) -> float:
        return max(-limit, min(limit, value))

    def compute_velocity(self) -> tuple[float, float, float]:
        now = time.monotonic()
        base_active = now - self.last_base_move_time <= self.command_timeout_sec
        if not base_active:
            return 0.0, 0.0, 0.0

        vx = (
            self.clip(self.latest_delta_pos * self.linear_gain, self.max_vx)
            if base_active
            else 0.0
        )
        vy = self.clip(self.latest_delta_lateral * self.lateral_gain, self.max_vy)
        vw = (
            self.clip(self.latest_delta_yaw * self.angular_gain, self.max_vw)
            if base_active
            else 0.0
        )
        return vx, vy, vw

    def send_speed(self, vx: float, vy: float, vw: float) -> None:
        if not self.control_enabled:
            return
        self.task_id += 1
        command = {
            "taskStartTimeStamp": int(time.time() * 1000),
            "taskGroupCmd": 1,
            "taskID": self.task_id,
            "actionType": 2,
            "speed": {"vx": vx, "vy": vy, "vw": vw},
        }
        try:
            payload = json.dumps(command, separators=(",", ":"))
            self.redis_client.set(self.redis_key_command, payload)
            self.get_logger().debug(f"发送 taskGroupCmd: {command}")
        except redis.RedisError as exc:
            self.get_logger().error(f"发送 taskGroupCmd 失败: {exc}")

    def command_timer_callback(self) -> None:
        if self.shutting_down:
            return
        vx, vy, vw = self.compute_velocity()
        self.send_speed(vx, vy, vw)

    def send_stop(self) -> None:
        for _ in range(self.stop_repeat_count):
            self.send_speed(0.0, 0.0, 0.0)
            time.sleep(self.command_interval_sec)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ChassisBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
