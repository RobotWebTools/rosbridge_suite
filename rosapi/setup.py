import os

from setuptools import find_packages, setup

package_name = "rosapi"

setup(
    name=package_name,
    version="1.3.2",
    packages=find_packages(exclude=["test"]),
    data_files=[
        # Install marker file in the package index
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        # Include our package.xml file
        (os.path.join("share", package_name), ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    author="Jonathan Mace",
    author_email="jonathan.c.mace@gmail.com",
    maintainer="Jihoon Lee, Foxglove",
    maintainer_email="jihoonlee.in@gmail.com, ros-tooling@foxglove.dev",
    description="Provides service calls for getting ros meta-information, like list of topics, services, params, etc.",
    license="BSD",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["rosapi_node = rosapi.rosapi_node:main"],
    },
)
