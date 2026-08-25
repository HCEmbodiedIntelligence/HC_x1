import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("hc_tj_camera_bridge")
    config_path = os.path.join(pkg_share, "config", "cameras.yaml")
    domain_id = os.environ.get("ROS_DOMAIN_ID", "14")
    localhost_only = os.environ.get("ROS_LOCALHOST_ONLY", "0")

    camera_driver = Node(
        package="camera_driver",
        executable="camera_driver",
        name="hc_tj_camera_driver",
        output="screen",
        emulate_tty=True,
        additional_env={
            "ROS_DOMAIN_ID": domain_id,
            "ROS_LOCALHOST_ONLY": localhost_only,
        },
    )

    head_compressor = Node(
        package="hc_tj_camera_bridge",
        executable="image_jpeg_compressor",
        name="camera_bridge_head",
        output="screen",
        parameters=[config_path],
        additional_env={
            "ROS_DOMAIN_ID": domain_id,
            "ROS_LOCALHOST_ONLY": localhost_only,
        },
    )

    head_depth_relay = Node(
        package="hc_tj_camera_bridge",
        executable="depth_png_compressor",
        name="camera_bridge_head_depth",
        output="screen",
        parameters=[config_path],
        additional_env={
            "ROS_DOMAIN_ID": domain_id,
            "ROS_LOCALHOST_ONLY": localhost_only,
        },
    )

    head_calibration_bridge = Node(
        package="hc_tj_camera_bridge",
        executable="camera_calibration_json_bridge",
        name="camera_bridge_head_calibration",
        output="screen",
        parameters=[config_path],
        additional_env={
            "ROS_DOMAIN_ID": domain_id,
            "ROS_LOCALHOST_ONLY": localhost_only,
        },
    )

    return LaunchDescription(
        [
            LogInfo(
                msg=[
                    "[hc_tj_camera_bridge] ROS_DOMAIN_ID=",
                    domain_id,
                    ", ROS_LOCALHOST_ONLY=",
                    localhost_only,
                ]
            ),
            LogInfo(
                msg=(
                    "[hc_tj_camera_bridge] camera_driver -> /cameras/eye/color -> "
                    "/io_teleop/camera_head/color; /cameras/eye/depth -> "
                    "/io_teleop/camera_head/depth (16-bit PNG); /cameras/eye/param -> "
                    "/io_teleop/camera_head/calibration (versioned JSON); robot-camera "
                    "static transforms -> /io_teleop/camera_head/extrinsics (versioned JSON)"
                )
            ),
            camera_driver,
            head_calibration_bridge,
        ]
    )
