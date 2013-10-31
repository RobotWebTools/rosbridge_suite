^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosbridge_library
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.1 (2013-10-31)
------------------
* Implement multiple subscriptions to latched topics (fixes `#1 <https://github.com/RobotWebTools/rosbridge_suite/issues/1>`_).
* generate more natural json for service call result
* add result field to service response
* Contributors: Siegfried-A. Gevatter Pujals, Takashi Ogura

0.5.0 (2013-07-17)
------------------
* 0.5.0 preparation for hydro release
* even more missing depends for unit tests
* more missing test packages
* missing depends added when running tests
* rostest now uses devel instead of install
* rostest added to package
* Contributors: Jihoon Lee, Russell Toris

0.4.4 (2013-04-08)
------------------

0.4.3 (2013-04-03 08:24)
------------------------

0.4.2 (2013-04-03 08:12)
------------------------
* eclipse projects removed
* Contributors: Russell Toris

0.4.1 (2013-03-07)
------------------
* adding message generation build dependency
* Contributors: Jihoon Lee

0.4.0 (2013-03-05)
------------------
* removing rostest
* Commenting out rostest
* Update rosbridge_library/package.xml
  removed <test_depend>rospy</test_depend>
* Fixes "'int' is not iterable" bug.
* Adds test_all.test launch file.
* Error fix from wrong package name.
* Moves test package tests into rosbridge_library.
  I learned about NOINSTALL for msg and srv generation in CMakeList.
* Resolves submodule issues.
* Uses only 1 .gitignore to avoid confusion.
* Merge pull request `#15 <https://github.com/RobotWebTools/rosbridge_suite/issues/15>`_ from baalexander/remove_unregister
  Removes buggy unregister call.
* Removes buggy unregister call.
  Fixes Issue `#12 <https://github.com/RobotWebTools/rosbridge_suite/issues/12>`_.
* Adds BSD license header to code files.
  See Issue `#13 <https://github.com/RobotWebTools/rosbridge_suite/issues/13>`_.
* Removing ultrajson from rosbridge.
  If JSON parsing becomes a performance bottle neck, we can readd it.
* Catkinizing rosbridge_library and server.
* PNG compression now creates a square RGB image padded with new-line characters
* Add stack dependencies and rosdeps.
* Collapse directory structure.
* Moved the packages inside a folder called rosbridge
* Initial commit of rosbridge_library
* Contributors: Austin Hendrix, Brandon Alexander, David Gossow, Jihoon Lee, Jonathan Mace, Russell Toris
