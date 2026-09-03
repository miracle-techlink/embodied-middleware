from setuptools import setup
from glob import glob
import os

package_name = "middleware"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name, package_name + ".nodes"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.json")),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="will",
    maintainer_email="will@example.com",
    description="reBot 单臂采集栈 ROS2 消息中心",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "rebot_arm_node = middleware.nodes.rebot_arm_node:main",
            "starai_leader_node = middleware.nodes.starai_leader_node:main",
            "teleop_map_node = middleware.nodes.teleop_map_node:main",
            "orbbec_node = middleware.nodes.orbbec_node:main",
            "uvc_node = middleware.nodes.uvc_node:main",
            "msg_center_bench = middleware.nodes.msg_center_bench:main",
            "piper_arm_node = middleware.nodes.piper_arm_node:main",
        ],
    },
)
