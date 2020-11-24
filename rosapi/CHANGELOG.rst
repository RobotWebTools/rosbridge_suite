^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosapi
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.11.10 (2020-09-08)
--------------------
* Fixed filter_globs for noetic (`#506 <https://github.com/RobotWebTools/rosbridge_suite/issues/506>`_)
  Service calls with non empty requests (e.g. /rosapi/topics_for_type) were crashing due to filter's return type in python 3.
* Contributors: foreignrobot

0.11.9 (2020-05-27)
-------------------

0.11.8 (2020-05-21)
-------------------

0.11.7 (2020-05-13)
-------------------

0.11.6 (2020-04-29)
-------------------

0.11.5 (2020-04-08)
-------------------
* Python 3 updates/fixes (`#460 <https://github.com/RobotWebTools/rosbridge_suite/issues/460>`_)
  * rosbridge_library, rosbridge_server: Update package format
  Add Python3 conditional dependencies where applicable.
  * rosbridge_library: Fix pngcompression for Python 3
  * rosapi: Use catkin_install_python for scripts
* Contributors: Alexey Rogachevskiy

0.11.4 (2020-02-20)
-------------------
* Add cbor-raw compression (`#452 <https://github.com/RobotWebTools/rosbridge_suite/issues/452>`_)
  The CBOR compression is already a huge win over JSON or PNG encoding,
  but it’s still suboptimal in some situations. This PR adds support for
  getting messages in their raw binary (ROS-serialized) format. This has
  benefits in the following cases:
  - Your application already knows how to parse messages in bag files
  (e.g. using [rosbag.js](https://github.com/cruise-automation/rosbag.js),
  which means that now you can use consistent code paths for both bags
  and live messages.
  - You want to parse messages as late as possible, or in parallel, e.g.
  only in the thread or WebWorker that cares about the message. Delaying
  the parsing of the message means that moving or copying the message to
  the thread is cheaper when its in binary form, since no serialization
  between threads is necessary.
  - You only care about part of the message, and don't need to parse the
  rest of it.
  - You really care about performance; no conversion between the ROS
  binary format and CBOR is done in the rosbridge_sever.
* Fix rosapi get_action_servers (`#429 <https://github.com/RobotWebTools/rosbridge_suite/issues/429>`_)
  The currently used proxy.get_topics function does not exists and results in the following error: `"AttributeError: 'module' object has no attribute 'get_topics'\n"`
  This change uses the existing `get_topics_and_types` method to get a list of topics.
* Contributors: Jan Paul Posma, Jørgen Borgesen

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
* Added services_glob to CallServices, added globs to rosbridge_tcp and rosbridge_udp, and other miscellanous fixes.
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
