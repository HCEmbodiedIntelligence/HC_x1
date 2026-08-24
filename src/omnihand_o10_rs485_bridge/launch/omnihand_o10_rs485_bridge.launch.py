from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    vr_trigger_arg = DeclareLaunchArgument(
        "enable_vr_trigger_control",
        default_value="false",
        description="Use left/right VR middle (grip) triggers to control the corresponding hand",
    )

    return LaunchDescription([
        vr_trigger_arg,
        Node(
            package="omnihand_o10_rs485_bridge",
            executable="omnihand_o10_rs485_bridge_node",
            name="omnihand_o10_rs485_bridge",
            output="screen",
            parameters=[{
                "left_uart_port": "/dev/ttyTHS2",
                "right_uart_port": "/dev/ttyTHS0",
                "uart_baudrate": 460800,
                "hand_device_id": 1,
                "enable_left": True,
                "enable_right": True,
                "enable_vibration_feedback": True,
                "vibration_topic": "/io_teleop/vibration_feedback_exo",
                "vibration_rate_hz": 10.0,
                "hand_joint_state_topic": "/io_teleop/hand_joint_states",
                "hand_joint_state_rate_hz": 20.0,
                "hand_trigger_topic": "/io_teleop/hand_trigger",
                "ignore_placeholder_finger_cmd": True,
                "placeholder_joint_threshold": 0.05,
                "enable_vr_trigger_control": ParameterValue(
                    LaunchConfiguration("enable_vr_trigger_control"), value_type=bool
                ),
                "vr_data_topic": "/io_teleop/vr_data",
                "vr_axis_min": 0.0,
                "vr_axis_max": 1.0,
                "vr_trigger_deadzone": 0.02,
                "vr_close_time_sec": 0.6,
                "vr_open_time_sec": 0.6,
                "vr_command_timeout_sec": 0.5,
            }],
        ),
    ])
