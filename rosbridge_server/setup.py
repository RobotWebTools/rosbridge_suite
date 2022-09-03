import os
from glob import glob
from setuptools import setup

package_name = "rosbridge_server"

setup(
    name=package_name,
    version="1.3.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name,
            [
                "launch/rosbridge_websocket_launch.py",
                "launch/rosbridge_websocket_launch.xml",
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    author="Jonathan Mace",
    author_email="jonathan.c.mace@gmail.com",
    maintainer="Jihoon Lee",
    maintainer_email="jihoonlee.in@gmail.com",
    description="A WebSocket interface to rosbridge.",
    license="BSD",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "rosbridge_websocket = rosbridge_server.rosbridge_websocket:main",
        ],
    },
)
