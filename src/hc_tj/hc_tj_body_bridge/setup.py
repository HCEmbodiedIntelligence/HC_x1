from setuptools import setup

package_name = "hc_tj_body_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/body.yaml"]),
        (
            "share/" + package_name + "/launch",
            ["launch/chassis_bridge.launch.py"],
        ),
    ],
    install_requires=["setuptools", "redis"],
    zip_safe=True,
    maintainer="io_teleop",
    maintainer_email="dev@io-ai.tech",
    description="hc_tj chassis Redis bridge for io_teleop",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "chassis_bridge_node = hc_tj_body_bridge.chassis_bridge_node:main",
        ],
    },
)
