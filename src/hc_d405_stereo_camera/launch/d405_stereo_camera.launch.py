import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory("hc_d405_stereo_camera")
    default_config = os.path.join(package_share, "config", "d405_stereo.yaml")

    config_arg = DeclareLaunchArgument(
        "config",
        default_value=default_config,
        description="Dual D405 parameter YAML",
    )
    left_serial_arg = DeclareLaunchArgument(
        "left_serial", default_value="260322275504"
    )
    right_serial_arg = DeclareLaunchArgument(
        "right_serial", default_value="260322276941"
    )
    left_enabled_arg = DeclareLaunchArgument(
        "left_enabled",
        default_value=EnvironmentVariable(
            "HC_D405_LEFT_ENABLED", default_value="true"
        ),
    )
    right_enabled_arg = DeclareLaunchArgument(
        "right_enabled",
        default_value=EnvironmentVariable(
            "HC_D405_RIGHT_ENABLED", default_value="true"
        ),
    )
    vr_topic_arg = DeclareLaunchArgument(
        "vr_topic", default_value="/io_teleop/camera_d405/vr/composite"
    )

    camera_node = Node(
        package="hc_d405_stereo_camera",
        executable="d405_stereo_camera_node",
        name="d405_stereo_camera",
        output="screen",
        emulate_tty=True,
        parameters=[
            LaunchConfiguration("config"),
            {
                "left.enabled": ParameterValue(
                    LaunchConfiguration("left_enabled"), value_type=bool
                ),
                "right.enabled": ParameterValue(
                    LaunchConfiguration("right_enabled"), value_type=bool
                ),
                "left.serial_no": ParameterValue(
                    LaunchConfiguration("left_serial"), value_type=str
                ),
                "right.serial_no": ParameterValue(
                    LaunchConfiguration("right_serial"), value_type=str
                ),
                "vr_output_topic": ParameterValue(
                    LaunchConfiguration("vr_topic"), value_type=str
                ),
            },
        ],
    )

    return LaunchDescription(
        [
            config_arg,
            left_serial_arg,
            right_serial_arg,
            left_enabled_arg,
            right_enabled_arg,
            vr_topic_arg,
            LogInfo(
                msg=[
                    "[hc_d405_stereo_camera] left=",
                    LaunchConfiguration("left_serial"),
                    " (enabled=",
                    LaunchConfiguration("left_enabled"),
                    "), right=",
                    LaunchConfiguration("right_serial"),
                    " (enabled=",
                    LaunchConfiguration("right_enabled"),
                    ")",
                    ", VR=",
                    LaunchConfiguration("vr_topic"),
                ]
            ),
            camera_node,
        ]
    )
