#!/usr/bin/env python
# $URL$
# $Rev$

# Build script for PyPNG.

# This is intended for PyPNG developers.  If you want to install PyPNG
# then you should use distutils and setup.py instead (``python setup.py
# install`` should do the trick).

# Currently only the documentation is built.  Nothing else requires
# building.

import os
os.system('rm -fr html')
os.system('sphinx-build -N -d sphinx-crud -a man html')
