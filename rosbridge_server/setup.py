import os
from glob import glob

from setuptools import find_packages, setup

package_name = "rosbridge_server"

setup(
    name=package_name,
    version="1.3.2",
    packages=find_packages(exclude=["test"]),
    data_files=[
        # Install marker file in the package index
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        # Include our package.xml file
        (os.path.join("share", package_name), ["package.xml"]),
        # Include all launch files.
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    author="Jonathan Mace",
    author_email="jonathan.c.mace@gmail.com",
    maintainer="Jihoon Lee, Foxglove",
    maintainer_email="jihoonlee.in@gmail.com, ros-tooling@foxglove.dev",
    description="A WebSocket interface to rosbridge.",
    license="BSD",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["rosbridge_websocket = rosbridge_server.rosbridge_websocket:main"],
    },
)
