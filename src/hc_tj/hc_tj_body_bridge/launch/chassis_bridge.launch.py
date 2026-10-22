from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("hc_tj_body_bridge")
    config_path = os.path.join(pkg_share, "config", "body.yaml")

    control_arg = DeclareLaunchArgument(
        "control",
        default_value="true",
        description="是否允许向底盘 Redis 下发速度；false 时仅上报底盘状态",
    )

    chassis_node = Node(
        package="hc_tj_body_bridge",
        executable="chassis_bridge_node",
        name="chassis_bridge",
        output="screen",
        parameters=[
            config_path,
            {
                "enable_control": ParameterValue(
                    LaunchConfiguration("control"), value_type=bool
                )
            },
        ],
    )

    return LaunchDescription([control_arg, chassis_node])
