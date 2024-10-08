^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosapi
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.1.0 (2024-10-08)
------------------
* Fix invalid import of get_parameter_value in rosapi for ROS2 Jazzy. (`#932 <https://github.com/RobotWebTools/rosbridge_suite/issues/932>`_)
* Contributors: David Oberacker

2.0.0 (2024-10-08)
------------------
* rosapi: Don't start parameter services that aren't spun (`#944 <https://github.com/RobotWebTools/rosbridge_suite/issues/944>`_)
* Handle ROS 2 types properly (`#883 <https://github.com/RobotWebTools/rosbridge_suite/issues/883>`_)
* Port unit tests to ROS 2 + Fix CBOR conversion and PNG compression (`#882 <https://github.com/RobotWebTools/rosbridge_suite/issues/882>`_)
* Contributors: Brad Martin, Scott Bell, Sebastian Castro

1.3.2 (2023-09-27)
------------------
* Fix ROS2 CI for iron & rolling (`#875 <https://github.com/RobotWebTools/rosbridge_suite/issues/875>`_)
* Contributors: Hans-Joachim Krauch

1.3.1 (2022-10-21)
------------------
* Add graceful shutdown (`#794 <https://github.com/RobotWebTools/rosbridge_suite/issues/794>`_)
* Contributors: Hans-Joachim Krauch

1.3.0 (2022-08-16)
------------------
* Fixed /get_param service for arrays (`#776 <https://github.com/RobotWebTools/rosbridge_suite/issues/776>`_)
* Contributors: p0rys

1.2.0 (2022-05-20)
------------------
* Added `/rosapi/get_ros_version` service (`#708 <https://github.com/RobotWebTools/rosbridge_suite/issues/708>`_)
* Fixed node name collision with websocket launch file (`#707 <https://github.com/RobotWebTools/rosbridge_suite/issues/707>`_)
* Contributors: Jacob Bandes-Storch, Kedus Mathewos, rob-clarke

1.1.2 (2022-01-03)
------------------

1.1.1 (2021-12-09)
------------------

1.1.0 (2021-10-22)
------------------
* Move msg/srv from rosapi and rosbridge_library into separate packages; enable Rolling in CI (`#665 <https://github.com/RobotWebTools/rosbridge_suite/issues/665>`_)
* Exit cleanly on SIGINT; remove sleep in test (`#667 <https://github.com/RobotWebTools/rosbridge_suite/issues/667>`_)
* Remove unused service_host and search_param services (`#660 <https://github.com/RobotWebTools/rosbridge_suite/issues/660>`_)
* Migrate remaining linters to pre-commit (`#657 <https://github.com/RobotWebTools/rosbridge_suite/issues/657>`_)
* Add pre-commit, format with black and isort (`#648 <https://github.com/RobotWebTools/rosbridge_suite/issues/648>`_)
* Contributors: Adrian Macneil, Jacob Bandes-Storch, Kenji Miyake

1.0.8 (2021-08-26)
------------------
* Add missing test_depends and buildtool_depends (`#617 <https://github.com/RobotWebTools/rosbridge_suite/issues/617>`_)
* Fix various Python code style and lint issues
* Contributors: Christian Clauss, Jacob Bandes-Storch

1.0.7 (2021-08-18)
------------------
* Load message definitions from .msg files; exclude /msg/ and include builtin_interfaces in combined definitions (`#597 <https://github.com/RobotWebTools/rosbridge_suite/issues/597>`_)
* Fix typos discovered by codespell (`#600 <https://github.com/RobotWebTools/rosbridge_suite/issues/600>`_)
* Contributors: Christian Clauss, Jacob Bandes-Storch

1.0.6 (2021-08-17)
------------------
* Include /msg/ in type names (`#591 <https://github.com/RobotWebTools/rosbridge_suite/issues/591>`_)
* Fix broken links in changelogs
* Contributors: Jacob Bandes-Storch

1.0.5 (2021-08-12)
------------------

1.0.4 (2021-08-11)
------------------
* Include /msg/ in type names (`#584 <https://github.com/RobotWebTools/rosbridge_suite/issues/584>`_)
  It's more canonical for ROS 2 type names to be of the form `foo_msgs/msg/Bar` rather than just `foo_msgs/Bar`. This is occasionally reflected in documentation and command line tooling: https://docs.ros.org/en/galactic/Tutorials/Topics/Understanding-ROS2-Topics.html#ros2-interface-show
  So rather than stripping out `/msg/`, we include it in the type definitions.
  See also: https://github.com/RobotWebTools/rosmsg/pull/12
* Contributors: Jacob Bandes-Storch

1.0.3 (2021-08-03)
------------------
* Add TopicsAndRawTypes service (`#574 <https://github.com/RobotWebTools/rosbridge_suite/issues/574>`_, adapted from ROS 1 implementation `#452 <https://github.com/RobotWebTools/rosbridge_suite/issues/452>`_)
* fix: remove json encoding before setting string params (`#521 <https://github.com/RobotWebTools/rosbridge_suite/issues/521>`_)
* Update rosapi/proxy.py to match eloquent API (`#447 <https://github.com/RobotWebTools/rosbridge_suite/issues/447>`_)
* Contributors: Jacob Bandes-Storch, justinscorringe, travipross

1.0.2 (2019-09-24)
------------------

1.0.1 (2019-09-20)
------------------
* fix missing dependency

1.0.0 (2019-09-19)
------------------
* Port to ROS 2

0.11.3 (2019-08-07)
-------------------
* Travis CI: Look for Python syntax errors and undefined name (`#420 <https://github.com/RobotWebTools/rosbridge_suite/issues/420>`_)
  * Travis CI: Look for Python syntax errors and undefined name
  _It would be prudent to start running the tests in both 2 and 3._  https://github.com/RobotWebTools/rosbridge_suite/issues/401#issuecomment-512069249
  * Add names to protect the guilty
  * Five jobs, not six
  * Identity is not the same thing as equality in Python
  * Flake8 tests now pass on Python 2
* Contributors: cclauss

0.11.2 (2019-07-08)
-------------------
* constnames and constvalues in typedef (`#412 <https://github.com/RobotWebTools/rosbridge_suite/issues/412>`_)
* Contributors: Kad91

0.11.1 (2019-05-08)
-------------------

0.11.0 (2019-03-29)
-------------------

0.10.2 (2019-03-04)
-------------------
* Use Master.getTopicTypes() in /rosapi/topics to increase performance (`#381 <https://github.com/RobotWebTools/rosbridge_suite/issues/381>`_)
* Contributors: Affonso, Guilherme

0.10.1 (2018-12-16)
-------------------

0.10.0 (2018-12-14)
-------------------
* Drop use of ros Python module (`#374 <https://github.com/RobotWebTools/rosbridge_suite/issues/374>`_)
* Fixes passing of globs to proxy (`#355 <https://github.com/RobotWebTools/rosbridge_suite/issues/355>`_)
  * Fixes handling and passing of globs to proxy
  * Removes some confusing imports
* Fix a few problems (`#350 <https://github.com/RobotWebTools/rosbridge_suite/issues/350>`_)
  * xrange is not available in Python3, range works for both Python versions
  * the variable v is undefined in search_param, comparing the implementation with the sibling functions I expect name to be the intended variable
  * The module udp_handler is using the Authentication service but wasn't importing the module
* use package format 2, remove unnecessary dependencies (`#348 <https://github.com/RobotWebTools/rosbridge_suite/issues/348>`_)
* Contributors: Anwar, Dirk Thomas, Jochen Sprickerhof

0.9.0 (2018-04-09)
------------------

0.8.6 (2017-12-08)
------------------
* Fixed action_servers filter to allow more than one namespace (`#305 <https://github.com/RobotWebTools/rosbridge_suite/issues/305>`_)
  * Modified action_servers filter to detect topics with more than one namespace
  * Fixed to return the full namespace
* Contributors: milesial

0.8.5 (2017-11-23)
------------------
* Add Python3 compatibility (`#300 <https://github.com/RobotWebTools/rosbridge_suite/issues/300>`_)
  * First pass at Python 3 compatibility
  * message_conversion: Only call encode on a Python2 str or bytes type
  * protocol.py: Changes for dict in Python3. Compatible with Python 2 too.
  * More Python 3 fixes, all tests pass
  * Move definition of string_types to rosbridge_library.util
* Contributors: Kartik Mohta

0.8.4 (2017-10-16)
------------------
* Handles empty globes properly (`#297 <https://github.com/RobotWebTools/rosbridge_suite/issues/297>`_)
  * Refactors get_globs function to a separate module
  * Refactors the filtering that uses the globs
  * Some linting
  * Handles topic types for empty globs
  * Refactors out an any_match function
  * Simplifies filter_action_servers
  * Imports socket for the errors
  * Uses import .glob_helper
* Contributors: Anwar

0.8.3 (2017-09-11)
------------------

0.8.2 (2017-09-11)
------------------
* Removes array delimiters while parsing parameters (`#292 <https://github.com/RobotWebTools/rosbridge_suite/issues/292>`_)
* Contributors: Anwar

0.8.1 (2017-08-30)
------------------

0.8.0 (2017-08-30)
------------------
* fix delete_param in rosapi (`#284 <https://github.com/RobotWebTools/rosbridge_suite/issues/284>`_)
* Merge pull request `#276 <https://github.com/RobotWebTools/rosbridge_suite/issues/276>`_ from sevenbitbyte/DOCUMENT_GLOB
  Document glob
* Update README.md
  Formatting and examples
* Create README.md
* Contributors: 7bit, Jihoon Lee

0.7.17 (2017-01-25)
-------------------
* Added bug fix in rosapi
* no rospy needed, just for debug logging
* new service: get actionlib servers
* adjust log level for security globs
  Normal operation (i.e. no globs or successful verification of requests) is now silent, with illegal requests producing a warning.
* correct default values for security globs
  also accept empty list as the default "do not check globs" value in addition to None.
  Finally, append rosapi service glob after processing command line input so it's not overwritten
* Added services_glob to CallServices, added globs to rosbridge_tcp and rosbridge_udp, and other miscellaneous fixes.
* As per the suggestions of @T045T, fixed several typos, improved logging, and made some style fixes.
* Fixed time object field definitions to match documentation.
* Two minor fixes.
* Added new parameters for topic and service security.
  Added 3 new parameters to rosapi and rosbridge_server which filter the
  topics, services, and parameters broadcast by the server to match an
  array of glob strings.
* Contributors: Devon Ash, Eric, Marco Arruda, Nils Berg

0.7.16 (2016-08-15)
-------------------
* new srv: topics types and details
* Contributors: Marco Arruda

0.7.15 (2016-04-25)
-------------------
* changelog updated
* Contributors: Russell Toris

0.7.14 (2016-02-11)
-------------------
* Update proxy.py
  Fixes an issue when call the service "/rosapi/service_type"
* Contributors: Robert Codd-Downey

0.7.13 (2015-08-14)
-------------------
* Fix catkin_lint issues
* Contributors: Matt Vollrath

0.7.12 (2015-04-07)
-------------------

0.7.11 (2015-03-23)
-------------------
* rename rosapi script to rosapi_node to address `#170 <https://github.com/RobotWebTools/rosbridge_suite/issues/170>`_
* Contributors: Jihoon Lee

0.7.10 (2015-02-25)
-------------------
* Make get_topics() and get_topic_type() reference the full list of active topics.
* Contributors: Justin Huang

0.7.9 (2015-02-24)
------------------
* add findding service function as specific service type
* Contributors: dwlee

0.7.8 (2015-01-16)
------------------

0.7.7 (2015-01-06)
------------------

0.7.6 (2014-12-26)
------------------
* 0.7.5
* update changelog
* 0.7.4
* changelog updated
* 0.7.3
* changelog updated
* 0.7.2
* changelog updated
* 0.7.1
* update changelog
* 0.7.0
* changelog updated
* Contributors: Jihoon Lee, Russell Toris

0.7.5 (2014-12-26)
------------------

0.7.4 (2014-12-16)
------------------

0.7.3 (2014-12-15)
------------------

0.7.2 (2014-12-15)
------------------
* 0.7.1
* update changelog
* Contributors: Jihoon Lee

0.7.1 (2014-12-09)
------------------

0.7.0 (2014-12-02)
------------------

0.6.8 (2014-11-05)
------------------

0.6.7 (2014-10-22)
------------------
* updated package manifests
* Contributors: Russell Toris

0.6.6 (2014-10-21)
------------------

0.6.5 (2014-10-14)
------------------
* 0.6.4
* update changelog
* 0.6.3
* update change log
* Contributors: Jihoon Lee

0.6.4 (2014-10-08)
------------------

0.6.3 (2014-10-07)
------------------

0.6.2 (2014-10-06)
------------------

0.6.1 (2014-09-01)
------------------
* make rosapis use absolute namespace
* Contributors: Jihoon Lee

0.6.0 (2014-05-23)
------------------
* Ensure proper locking for Parameter Server access
* Contributors: Lasse Rasinen

0.5.4 (2014-04-17)
------------------
* add rosnode and rosgraph
* Contributors: Jihoon Lee

0.5.3 (2014-03-28)
------------------

0.5.2 (2014-03-14)
------------------

0.5.1 (2013-10-31)
------------------

0.5.0 (2013-07-17)
------------------
* 0.5.0 preparation for hydro release
* Removes trailing commas.
* removing global bin installation in setup.py
* Contributors: Brandon Alexander, Jihoon Lee

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
* fixes import issue in rosapi
* Contributors: Russell Toris

0.4.0 (2013-03-05)
------------------
* Fixes ambiguous params class reference.
* Uses only 1 .gitignore to avoid confusion.
* Fixing rosapi's "Cannot include proxy..." errors.
* Adds BSD license header to code files.
  See Issue `#13 <https://github.com/RobotWebTools/rosbridge_suite/issues/13>`_.
* rosbridge_server requires rosapi.
* Adds message and service generation to rosapi.
* Adding setup.py to rosapi.
* Clarifies name of rosapi is rosapi.
* Catkinizes rosapi.
* Collapse directory structure.
* Contributors: Austin Hendrix, Brandon Alexander
