import os
from glob import glob
from setuptools import setup

package_name = "rosbridge_library"

setup(
    name=package_name,
    version="1.3.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    author="Jonathan Mace",
    author_email="jonathan.c.mace@gmail.com",
    maintainer="Jihoon Lee",
    maintainer_email="jihoonlee.in@gmail.com",
    description="The core rosbridge package, responsible for interpreting JSON andperforming the appropriate ROS action, like subscribe, publish, call service, and interact with params.",
    license="BSD",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)
