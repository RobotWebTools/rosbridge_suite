^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosbridge_suite
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.3 (2014-03-28)
------------------

0.5.2 (2014-03-14)
------------------

0.5.1 (2013-10-31)
------------------

0.5.0 (2013-07-17)
------------------
* 0.5.0 preparation for hydro release
* Contributors: Jihoon Lee

0.4.4 (2013-04-08)
------------------
* adding russl and myself as maintainer. adding build_tool depend
* Contributors: Jihoon Lee

0.4.3 (2013-04-03 08:24)
------------------------
* adding CMake list for meta pkg
* Contributors: Jihoon Lee

0.4.2 (2013-04-03 08:12)
------------------------

0.4.1 (2013-03-07)
------------------

0.4.0 (2013-03-05)
------------------
* cleaning up meta package
* Catkinizing rosbridge_library and server.
* Collapse directory structure.
* Removed print statements and also made sure to cast any tuples to lists.
* Removed the pypng dependency and finalised PIL dependency
* Use python imaging library to encode PNG instead of pypng
* Added the ujson library, modified cmakelists to install ujson to the
  user python directory.
* Fixed an inconsequential elif bug.
* Refactored to use simplejson if the package is installed.
* Added simplejson library and moved the location of the libraries.
* Temporary commit adding profiling messages. something is goign awry.
* Renamed rosbridge stack to rosbridge_suite
* Contributors: Austin Hendrix, Brandon Alexander, Jihoon Lee, jon
