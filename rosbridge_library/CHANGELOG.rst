^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosbridge_library
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.11.10 (2020-09-08)
--------------------
* possible fix for error when working with RosSharp, TypeError: can only concatenate str (not bytes) to str (`#514 <https://github.com/RobotWebTools/rosbridge_suite/issues/514>`_)
  Co-authored-by: Dmitri <dmitri@dmitri.com>
* Contributors: Dmitri

0.11.9 (2020-05-27)
-------------------
* noetic tests and fixes (`#503 <https://github.com/RobotWebTools/rosbridge_suite/issues/503>`_)
* Contributors: Matt Vollrath

0.11.8 (2020-05-21)
-------------------

0.11.7 (2020-05-13)
-------------------
* Fix backpressure deadlock (`#496 <https://github.com/RobotWebTools/rosbridge_suite/issues/496>`_)
  * Don't block Subscription.unregister()
  * Don't add messages to finished queue handler
  * Decouple incoming WS handling from server thread
* Contributors: Matt Vollrath

0.11.6 (2020-04-29)
-------------------

0.11.5 (2020-04-08)
-------------------
* Add script for dockerized development shell (`#479 <https://github.com/RobotWebTools/rosbridge_suite/issues/479>`_)
  * Add script for dockerized development shell
  * Fix queue dropping test
* Subscriber concurrency review (`#478 <https://github.com/RobotWebTools/rosbridge_suite/issues/478>`_)
  * Lock access to SubscriberManager public methods
  Prevent subscribe during unsubscribe critical section.
  * Unsubscribing an unsubscribed topic is an error
  This branch must not be ignored.
  * Cleanup some redundant syntax in subscribers impl
* Fix queue blocking (`#464 <https://github.com/RobotWebTools/rosbridge_suite/issues/464>`_)
  * Unblock QueueMessageHandler.handle_message
  The thread was holding the lock while pushing to a potentially blocking
  function.  Rewrite the logic and use a deque while we're at it.
  * Add test for subscription queue behavior
  Guarantee that the queue drops messages when blocked.
* Python 3 updates/fixes (`#460 <https://github.com/RobotWebTools/rosbridge_suite/issues/460>`_)
  * rosbridge_library, rosbridge_server: Update package format
  Add Python3 conditional dependencies where applicable.
  * rosbridge_library: Fix pngcompression for Python 3
  * rosapi: Use catkin_install_python for scripts
* Fixing wrong header/stamp in published ROS-messsages (`#472 <https://github.com/RobotWebTools/rosbridge_suite/issues/472>`_)
  When publishing a message to ROS (i.e. incoming from rosbridge_server's perspective), timestamps in the Header attributes all point to the same Time object iff the message contains multiple Header attributes (typically the case if a ROS message contains other ROS messages, e.g. ...Array-types) and rosparam use_sim_time is true.
* Contributors: Alexey Rogachevskiy, Matt Vollrath, danielmaier

0.11.4 (2020-02-20)
-------------------
* Concurrency review (`#458 <https://github.com/RobotWebTools/rosbridge_suite/issues/458>`_)
  * Safer locking in PublisherConsistencyListener
  * Safer locking in ros_loader
  * Print QueueMessageHandler exceptions to stderr
  * Register before resuming outgoing valve
  * Don't pause a finished socket valve
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
* Fix typos in rosbridge_library's description (`#450 <https://github.com/RobotWebTools/rosbridge_suite/issues/450>`_)
* Python 3 fix for dict::values (`#446 <https://github.com/RobotWebTools/rosbridge_suite/issues/446>`_)
  Under Python 3, values() returns a view-like object, and because that object is used outside the mutex, we were getting `RuntimeError: dictionary changed size during iteration` under some circumstances. This creates a copy of the values, restoring the Python 2 behaviour and fixing the problem.
* Contributors: Jan Paul Posma, Matt Vollrath, Mike Purvis, miller-alex

0.11.3 (2019-08-07)
-------------------

0.11.2 (2019-07-08)
-------------------

0.11.1 (2019-05-08)
-------------------
* fixed logwarn msg formatting in publishers (`#398 <https://github.com/RobotWebTools/rosbridge_suite/issues/398>`_)
* Contributors: Gautham P Das

0.11.0 (2019-03-29)
-------------------
* BSON can send Nan and Inf (`#391 <https://github.com/RobotWebTools/rosbridge_suite/issues/391>`_)
* Contributors: akira_you

0.10.2 (2019-03-04)
-------------------
* Fix typo (`#379 <https://github.com/RobotWebTools/rosbridge_suite/issues/379>`_)
* Contributors: David Weis

0.10.1 (2018-12-16)
-------------------
* Inline cbor library (`#377 <https://github.com/RobotWebTools/rosbridge_suite/issues/377>`_)
  Prefer system version with C speedups, but include pure Python implementation.
* Contributors: Matt Vollrath

0.10.0 (2018-12-14)
-------------------
* CBOR encoding (`#364 <https://github.com/RobotWebTools/rosbridge_suite/issues/364>`_)
  * Add CBOR encoding
  * Fix value extraction performance regression
  Extract message values once per message.
  * Fix typed array tags
  Was using big-endian tags and encoding little-endian.
  Always use little-endian for now since Intel is prevalent for desktop.
  Add some comments to this effect.
  * Update CBOR protocol documentation
  More information about draft typed arrays and when to use CBOR.
  * Fix 64-bit integer CBOR packing
  Use an actual 64-bit format.
* use package format 2, remove unnecessary dependencies (`#348 <https://github.com/RobotWebTools/rosbridge_suite/issues/348>`_)
* removing has_key for python3, keeping backwards compatibility (`#337 <https://github.com/RobotWebTools/rosbridge_suite/issues/337>`_)
  * removing has_key for python3, keeping backwards compatibility
  * py3 change for itervalues, keeping py2 compatibility
* Contributors: Andreas Klintberg, Dirk Thomas, Matt Vollrath

0.9.0 (2018-04-09)
------------------
* Fix typo in function call
* Add missing argument to InvalidMessageException (`#323 <https://github.com/RobotWebTools/rosbridge_suite/issues/323>`_)
  Add missing argument to InvalidMessageException constructor
* Make unregister_timeout configurable (`#322 <https://github.com/RobotWebTools/rosbridge_suite/issues/322>`_)
  Pull request `#247 <https://github.com/RobotWebTools/rosbridge_suite/issues/247>`_ introduces a 10 second delay to mitigate issue `#138 <https://github.com/RobotWebTools/rosbridge_suite/issues/138>`_.
  This change makes this delay configurable by passing an argument either
  on the command line or when including a launch file.
  Usage example:
  ```xml
  <launch>
  <include file="$(find rosbridge_server)/launch/rosbridge_websocket.launch">
  <arg name="unregister_timeout" value="5.0"/>
  </include>
  </launch>
  ```
  Closes `#320 <https://github.com/RobotWebTools/rosbridge_suite/issues/320>`_
* message_conversion: create stand-alone object inst (`#319 <https://github.com/RobotWebTools/rosbridge_suite/issues/319>`_)
  Catching the ROSInitException allows to create object
  instances without an initialized ROS state
* Fixes `#313 <https://github.com/RobotWebTools/rosbridge_suite/issues/313>`_ by fixing has_binary in protocol.py (`#315 <https://github.com/RobotWebTools/rosbridge_suite/issues/315>`_)
  * Fixes `#313 <https://github.com/RobotWebTools/rosbridge_suite/issues/313>`_ by fixing has_binary in protocol.py
  Checks for lists that have binary content as well as dicts
  * Minor refactoring for protocol.py
* fix fragment bug (`#316 <https://github.com/RobotWebTools/rosbridge_suite/issues/316>`_)
  * fix bug that lost data while sending large packets
  * fixed travis ci failed by @T045T
  * fixed travis ci failed by @T045T
  * travis ci failed
  * fix rosbridge_library/test/experimental/fragmentation+srv+tcp test bug
  * sync .travis.yaml
  * fix the service_response message bug
  * fix the fragment paring error
  * fix indentation of "service" line
* add graceful_shutdown() method to advertise_service capability
  This gives the service a bit of time to cancel any in-flight service requests (which should fix `#265 <https://github.com/RobotWebTools/rosbridge_suite/issues/265>`_).
  This is important because we busy-wait for a rosbridge response for service calls and those threads do not get stopped otherwise.
  Also, rospy service clients do not currently support timeouts, so any clients would be stuck too.
  A new test case in test_service_capabilities.py verifies the fix works
* Add rostest for service capabilities and fix bugs
  also fixed some typos
* Fix Travis config (`#311 <https://github.com/RobotWebTools/rosbridge_suite/issues/311>`_)
  * fix Travis config
  dist: kinetic is currently unsupported
  * fix rostests
  for some reason, rostest seems to hide the rosout node - changed tests to use other services
* Contributors: Anwar, Johannes Rothe, Jørgen Borgesen, Nils Berg, Phil, WH-0501, elgarlepp

0.8.6 (2017-12-08)
------------------
* Import StringIO from StringIO if python2 and from io if python3 fixes `#306 <https://github.com/RobotWebTools/rosbridge_suite/issues/306>`_ (`#307 <https://github.com/RobotWebTools/rosbridge_suite/issues/307>`_)
* Contributors: Jihoon Lee

0.8.5 (2017-11-23)
------------------
* Raise if inappropriate bson module is installed (Appease `#198 <https://github.com/RobotWebTools/rosbridge_suite/issues/198>`_) (`#270 <https://github.com/RobotWebTools/rosbridge_suite/issues/270>`_)
  * Raise Exception if inappropriate bson module is installed (Related to `#198 <https://github.com/RobotWebTools/rosbridge_suite/issues/198>`_)
* Add Python3 compatibility (`#300 <https://github.com/RobotWebTools/rosbridge_suite/issues/300>`_)
  * First pass at Python 3 compatibility
  * message_conversion: Only call encode on a Python2 str or bytes type
  * protocol.py: Changes for dict in Python3. Compatible with Python 2 too.
  * More Python 3 fixes, all tests pass
  * Move definition of string_types to rosbridge_library.util
* Contributors: Junya Hayashi, Kartik Mohta

0.8.4 (2017-10-16)
------------------

0.8.3 (2017-09-11)
------------------
* Type conversion convention correction, correcting issue `#240 <https://github.com/RobotWebTools/rosbridge_suite/issues/240>`_
* Contributors: Alexis Paques

0.8.2 (2017-09-11)
------------------

0.8.1 (2017-08-30)
------------------
* remove ujson from dependency to build in trusty (`#290 <https://github.com/RobotWebTools/rosbridge_suite/issues/290>`_)
* Contributors: Jihoon Lee

0.8.0 (2017-08-30)
------------------
* Cleaning up travis configuration (`#283 <https://github.com/RobotWebTools/rosbridge_suite/issues/283>`_)
  configure travis to use industial ci configuration. Now it uses xenial and kinetic
* Merge pull request `#272 <https://github.com/RobotWebTools/rosbridge_suite/issues/272>`_ from ablakey/patch-1
  Prevent a KeyError when bson_only_mode is unset.
* Update protocol.py
  Prevent a KeyError when bson_only_mode is unset.
* Merge pull request `#257 <https://github.com/RobotWebTools/rosbridge_suite/issues/257>`_ from Sanic/bson-only-mode
  Implemented a bson_only_mode flag for the TCP version of rosbridge
* Merge pull request `#247 <https://github.com/RobotWebTools/rosbridge_suite/issues/247>`_ from v-lopez/develop
  Delay unregister to mitigate `#138 <https://github.com/RobotWebTools/rosbridge_suite/issues/138>`_
* Change class constant to module constant
* Reduce timeout for tests
  Tests will sleep for 10% extra of the timeout to prevent some situations
  were the test sleep ended right before the unregister timer fired
* Fix test advertise errors after delayed unregister changes
* Fix missing tests due to delayed unregistration
* Move UNREGISTER_TIMEOUT to member class so it's accessible from outside
* minor change in variable usage
* Implemented a bson_only_mode flag for the TCP version of rosbridge; This allows you to switch to a full-duplex transmission of BSON messages and therefore eliminates the need for a base64 encoding of binary data; Use the new mode by starting:'roslaunch rosbridge_server rosbridge_tcp.launch bson_only_mode:=True' or passing '--bson_only_mode' to the rosbridge_tcp.py script
* Delay unregister to mitigate !138
* Contributors: Andrew Blakey, Jihoon Lee, Nils Berg, Patrick Mania, Victor Lopez

0.7.17 (2017-01-25)
-------------------
* adjust log level for security globs
  Normal operation (i.e. no globs or successful verification of requests) is now silent, with illegal requests producing a warning.
* add missing import
* correct default values for security globs
  also accept empty list as the default "do not check globs" value in addition to None.
  Finally, append rosapi service glob after processing command line input so it's not overwritten
* Added services_glob to CallServices, added globs to rosbridge_tcp and rosbridge_udp, and other miscellanous fixes.
* As per the suggestions of @T045T, fixed several typos, improved logging, and made some style fixes.
* Added new parameters for topic and service security.
  Added 3 new parameters to rosapi and rosbridge_server which filter the
  topics, services, and parameters broadcast by the server to match an
  array of glob strings.
* Contributors: Eric, Nils Berg

0.7.16 (2016-08-15)
-------------------
* Fixed deprecated code in pillow
* Contributors: vladrotea

0.7.15 (2016-04-25)
-------------------
* changelog updated
* Contributors: Russell Toris

0.7.14 (2016-02-11)
-------------------
* Another fix for code
* Replaced += with ''.join() for python code
* Default Protocol delay_between_messages = 0
  This prevents performance problems when multiple clients are subscribing to high frequency topics.
  Fixes `#203 <https://github.com/RobotWebTools/rosbridge_suite/issues/203>`_
* Contributors: Matt Vollrath, kiloreux

0.7.13 (2015-08-14)
-------------------
* Nevermind o_O
* Add test_depend too (just in case)
* Add dependency on python bson
* Get parameter at encode time
* Add flag for using the bson encoding
* revert comment regarding unpublisher
* avoiding racing condition
* Add bson encoding to the server side
* Fix catkin_lint issues
* don't unregister topic from rosbridge. It creates md5 sum warning.. #138
* Contributors: David Lu, Jihoon Lee, Matt Vollrath, dwlee

0.7.12 (2015-04-07)
-------------------
* use <test_depend> for test dependencies
* use rospy.resolve_name for namespaced service calls
* fix resolving namespaced service calls
* Contributors: Ramon Wijnands

0.7.11 (2015-03-23)
-------------------

0.7.10 (2015-02-25)
-------------------

0.7.9 (2015-02-24)
------------------

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
* rewrite of advertise service
* cleanup init function
* matches original call_service
* matches original call_service
* service_request --> reuse of call_service (previously defined)
* stop_service --> unadvertise_service
* service_name --> service
* service_type --> type
* removed service_module
* request_id --> id
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
* rewrite of advertise service
* cleanup init function
* matches original call_service
* matches original call_service
* service_request --> reuse of call_service (previously defined)
* stop_service --> unadvertise_service
* service_name --> service
* service_type --> type
* removed service_module
* request_id --> id
* Contributors: Russell Toris

0.6.8 (2014-11-05)
------------------
* add a lock to calls to load_manifest - apparently, it's not thread safe
  fixes #103 and #108
* Contributors: Nils Berg

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
* modify tests
  less duplicated code, some other changes to (hopefully) improve reliability. Tested locally about 30 times without encountering any failures.
* Change the behavior of MessageHandler.transition()
  Now reflects usage in the tests, i.e. a QueueMessageHandler only needs queue_length to be defined, not throttle_rate.
* 0.6.3
* update change log
* install util python module to fix #128
* Contributors: Jihoon Lee, Nils Berg

0.6.4 (2014-10-08)
------------------

0.6.3 (2014-10-07)
------------------
* install util python module to fix `#128 <https://github.com/RobotWebTools/rosbridge_suite/issues/128>`_
* Contributors: Jihoon Lee

0.6.2 (2014-10-06)
------------------
* Remove unused json imports; move json imports to utility
  Fixes #7
* Contributors: Graeme Yeates

0.6.1 (2014-09-01)
------------------
* Handle float infinity and NAN s
* Windows-related fix for PIL Image module import
* Fixed typo in raising type errors.
* something messed up indentation
  not sure how that could happen, worked here.
* map Inf and NaN to null
  JSON does not support Inf and NaN values. Currently they are just written into the JSON and JSON.parse on the client side will fail. Correct is to map them to null which will then be parsed correctly by JSON.parse on the client side.
  The issue with that is that the shortcut for lists of floats might be impossible (maybe someone else with more experience in python comes up with something else?). Maybe something similar is necessary in the to_inst case, but I can not really test them.
  Real world application is to process laser scans, they contain inf and nan values for some drivers if the measurements are invalid or out of range.
* Update .travis.yml and package.xml for rosbridge_library tests
* Put back unregister for the publisher and clarify the reconnect behavior
  of the test case. The exponential backoff of the client causes hard to
  understand timing of the events.
  All specs passed locally on hydro:
  SUMMARY
  * RESULT: SUCCESS
  * TESTS: 103
  * ERRORS: 0
  * FAILURES: 0
* Add copyright notice to the file
* Remove extra whitespace
* Make the test more deterministic
* Remove circular dependency.
* Contributors: Achim Konigs, Alex Sorokin, Alexander Sorokin, Jonathan Wade, jon-weisz

0.6.0 (2014-05-23)
------------------
* Ensure that service name is a string
  Closes `#104 <https://github.com/RobotWebTools/rosbridge_suite/issues/104>`_
* Contributors: Piyush Khandelwal

0.5.4 (2014-04-17)
------------------
* removing wrong import
* test case for fixed size of uint8 array
* uses regular expresion to match uint8 array and char array.
* logerr when it fails while message_conversion
* Contributors: Jihoon Lee

0.5.3 (2014-03-28)
------------------
* use queue_size for publishers
* Contributors: Jon Binney

0.5.2 (2014-03-14)
------------------
* First attempt adding latching support for topic publishers
* merging changes of groovy-devel into hydro-devel
* adding missing dependency in rosbridge_library `#70 <https://github.com/RobotWebTools/rosbridge_suite/issues/70>`_
* Fixed wrong unicode encoding
* support publishing non-ascii letters
* Added error message on result=False
  When call_service returns False as result, values contains the error message.
* added parameter lookup to rosbridge_tcp.py, modules where those are used, and default parameters to launch file; internal default-values still get used when launch-file does not provide them; internal defaults can be changed within rosbridge_tcp.py
* Merge branch 'experimental_branch' into new_features
* fix handling of partial/multiple/broken json by avoiding to pass nested json (without op-field) to rosbridge.. probably still needs more complex handling of incoming 'broken' json
* nested service not MiRoR related anymore
* added singleton for request-list; allows provider to send service response without specifying module and type, they get looked up when response is received via request_id
* fix for nested service responses - use ros_loader and message_conversion for populating an according instance
* use message_conversion in handle_servie_request
* snapshot for branch to show to genpy devs
* using float64 instead of std_msgs/Float64 lets scripts run fine.. ; next: fix with using std_msgs/Float64 --> need nested data field
* nested srv uses now message_conversion.extract_values
* adapted test scripted to ros_loader; (removed .srv from module_name
* use rosloader for finding service_class
* fixed calculation of fragment_count
* cleanup: files, notes, some code
* added message_field <message_intervall> to allow client to control delay between messages from rosbridge
* added TODO: check if service successfully registered in ros
* ..
* ..
* added description of new opcodes
* tests, comments, description, ..
* tested rosbridge_websocket with new capabilities; websocket test scripts not working yet..; but new caps are working when using rosbridge_websocket and tcp2ws wrapper --> so only testscripts need to be fixed for websockets.
* updated websocket test service server and client script to use websocket
* updated websocket test service server script to use websocket
* added files to test new caps with websocket server
* feierabend.. morgen weiter mit server & client JSON-decoder, see notes
* fixed parsing of incomplete/multiple JSON in incoming buffer; so clients do not need to use an intervall when sending to rosbridge
* only current changes; not yet done..
* code cleanup, not yet finished..; rosbridge logging much cleaner now
* fixed test_server_defragment - recodegit status
* minor
* linuxonandroid
* fixed some parts; ..still better do some redesign for queueing of messages..
* forced tcp_send to use queue and use delay between sends
* blocking behavior for service requests to non-ros; test-scripts use get-ip4 helper function; ..needs a lot cleanup before next steps..
* need to implement server side blocking of multiple requests, to keep implementation of service provider as easy and simple as possible
* not finished
* some changes.. still needs serveral fixes
* unique request_ids
* fixed deserialization of multiple fragments in incoming-data; was caused by too short delay between socket-sends (<0.2 seconds); maybe only temp. fixed
* added fragment sorting to test-client and test-server
* message_size debugging; TODO: sort list of received fragments! ; make sure receive_buffers are big enough for fragment_size + header..
* minor changes
* testing: service server fragmentsizes receive: 1  send: 1; client fragmentsize receive: 1; is working..
* fixed an error that caused service_response to appear quoted as string once too often; should be ok now
* fragmentation basically working; service_server can request fragmented service_calls, service_client can request fragmented responses; fragmentation can be requested by adding fragmentation_size parameter to any message sent to rosbridge
* some code cleanup
* set service_request_timeout back to 60 seconds; had 2s from timeout_tests..
* fixed example: non-ros_service_server.py to use only 1 socket; commented and structured code and comments in test-scripts
* some minor changes: comments, debug-output, ..
* added test script for non-ros_service_client calling service from non-ros_service_server
* added msg and srv files
* fixed (removed) dependency to beginner_tutorials for service_server test-scripts. beginner_tutorials package not needed anymore.
* behaviour on advertising existing service: replace service-provider, similar to ROS-groovy behaviour, see issues..
* behaviour on advertising existing service: replace service-provider, similar to ROS-groovy behaviour, see issues..
* removed obsolete test-scripts
* stop service added
* first working classes: service_server
* should use its own branch: service_server.py;  add initial thoughts and code-base for developing ServiceServer capability
* fixed errors in protocol.py and defragmentation.py
* added test-scripts for defragmentation AND tcp-server
* change json imports to try to use ujson or simplejson
* change json imports to try to use ujson or simplejson; correct log_message to show length of content/data instead of overall length
* fixed variable name in finish()
* Clean up of defragmentation.py.
* add defragmentation capability
* merge with fuerte-devel
* add defragmentation capability
* commented out that problematic unregister line
* Contributors: Brandon Alexander, Jihoon Lee, Julian Cerruti, Kaijen Hsiao, Stefan Profanter, dave, furushchev, fxm-db, ipa-fxm, root, unknown

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
