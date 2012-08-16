# $URL$
# $Rev$

# PyPNG setup.py
# This is the setup.py script used by distutils.

# You can install the png module into your Python distribution with:
# python setup.py install
# You can also do other standard distutil type things, but you can refer
# to the distutil documentation for that.

# This script is also imported as a module by the Sphinx conf.py script
# in the man directory, so that this file forms a single source for
# metadata.

# http://docs.python.org/release/2.4.4/lib/module-sys.html
import sys

conf = dict(
    name='pypng',
    version='0.0.13',
    description='Pure Python PNG image encoder/decoder',
    long_description="""
PyPNG allows PNG image files to be read and written using pure Python.

It's available from Google code:
http://code.google.com/p/pypng/downloads/list

Documentation is kindly hosted at python.org:
http://packages.python.org/pypng/
(and also available in the download tarball).
""",
    author='David Jones',
    author_email='drj@pobox.com',
    url='http://code.google.com/p/pypng/',
    package_dir={'':'code'},
    py_modules=['png'],
    classifiers=[
      'Topic :: Multimedia :: Graphics',
      'Topic :: Software Development :: Libraries :: Python Modules',
      'Programming Language :: Python',
      'Programming Language :: Python :: 2.3',
      'Programming Language :: Python :: 3',
      'License :: OSI Approved :: MIT License',
      'Operating System :: OS Independent',
      ],
    )
conf['download_url'] = \
  'http://pypng.googlecode.com/files/%(name)s-%(version)s.tar.gz' % conf

if sys.version_info >= (3,):
    conf['use_2to3'] = True

if __name__ == '__main__':
    try:
        # http://peak.telecommunity.com/DevCenter/setuptools#basic-use
        from setuptools import setup
    except ImportError:
        # http://docs.python.org/release/2.4.4/dist/setup-script.html
        from distutils.core import setup
    setup(**conf)
