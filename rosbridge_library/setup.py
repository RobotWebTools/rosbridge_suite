#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['rosbridge_library', 'rosbridge_library.internal', 'rosbridge_library.capabilities', 'rosbridge_library.util'],
    package_dir={'' : 'src'},
)

setup(**d)
