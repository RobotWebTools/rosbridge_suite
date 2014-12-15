#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['rosbridge_tools', 'rosbridge_tools.tornado', 'rosbridge_tools.tornado.platform', 'rosbridge_tools.backports', 'rosbridge_tools.backports.ssl_match_hostname'],
    package_dir={'': 'src'}
)

setup(**d)
