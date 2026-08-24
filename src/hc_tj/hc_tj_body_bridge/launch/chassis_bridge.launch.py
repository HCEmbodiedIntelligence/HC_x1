from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("hc_tj_body_bridge")
    config_path = os.path.join(pkg_share, "config", "body.yaml")

    chassis_node = Node(
        package="hc_tj_body_bridge",
        executable="chassis_bridge_node",
        name="chassis_bridge",
        output="screen",
        parameters=[config_path],
    )

    return LaunchDescription([chassis_node])
