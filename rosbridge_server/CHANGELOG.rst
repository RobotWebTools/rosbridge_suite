^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosbridge_server
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.8.6 (2017-12-08)
------------------

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

0.8.2 (2017-09-11)
------------------

0.8.1 (2017-08-30)
------------------

0.8.0 (2017-08-30)
------------------
* Merge pull request `#281 <https://github.com/RobotWebTools/rosbridge_suite/issues/281>`_ from RobotWebTools/expose_b64
  expose binary_encoder rosparam that was hidden in deep depth
* address review comment. more explicitly describe valid args
* correct the possible argument
* expose binary_encoder rosparam that was hidden in deep depth
* Merge pull request `#277 <https://github.com/RobotWebTools/rosbridge_suite/issues/277>`_ from T045T/remove_nodelay_for_udp
  don't try to set TCP nodelay option for UDP
* don't try to set TCP nodelay option for UDP
* Merge pull request `#273 <https://github.com/RobotWebTools/rosbridge_suite/issues/273>`_ from Sanic/set_bson_only_flags
  Set default for bson_only_mode in websocket handler and launch file.
* Set default for bson_only_mode in websocket handler and launch file.
* Merge pull request `#257 <https://github.com/RobotWebTools/rosbridge_suite/issues/257>`_ from Sanic/bson-only-mode
  Implemented a bson_only_mode flag for the TCP version of rosbridge
* Implemented a bson_only_mode flag for the TCP version of rosbridge; This allows you to switch to a full-duplex transmission of BSON messages and therefore eliminates the need for a base64 encoding of binary data; Use the new mode by starting:'roslaunch rosbridge_server rosbridge_tcp.launch bson_only_mode:=True' or passing '--bson_only_mode' to the rosbridge_tcp.py script
* Contributors: Adolfo Rodriguez Tsouroukdissian, Jihoon Lee, Nils Berg, Patrick Mania, pmania

0.7.17 (2017-01-25)
-------------------
* Fixed the launch files for the tcp and udp service. Without these modifications, the rosapi node fails because some rosparams are not defined properly before. Now the launchfiles comply to the websocket version.
* Added default topics to all launch files, and fixed bug where it would crash if nothing was put into the lists as values
* Fix: Set default to publish all topics
  Without better doc, one does not understand why no topics are published. I thought, something is broken.
  With this defaults, everything is working out of the box. And for a more secure setup, one can change it.
* correct default values for security globs
  also accept empty list as the default "do not check globs" value in addition to None.
  Finally, append rosapi service glob after processing command line input so it's not overwritten
* add missing imports and correct default values for glob parameters
* Added services_glob to CallServices, added globs to rosbridge_tcp and rosbridge_udp, and other miscellanous fixes.
* Two minor fixes.
* Added new parameters for topic and service security.
  Added 3 new parameters to rosapi and rosbridge_server which filter the
  topics, services, and parameters broadcast by the server to match an
  array of glob strings.
* Contributors: Devon Ash, Eric, Nils Berg, Patrick Mania, plieningerweb

0.7.16 (2016-08-15)
-------------------

0.7.15 (2016-04-25)
-------------------
* Track Twisted run_depend
  Fixes `#218 <https://github.com/RobotWebTools/rosbridge_suite/issues/218>`_
* Add rosbridge_udp cmake install rule `#225 <https://github.com/RobotWebTools/rosbridge_suite/issues/225>`_
* Stop UDP server on ROS shutdown
* changelog updated
* Track Twisted run_depend
  Fixes `#218 <https://github.com/RobotWebTools/rosbridge_suite/issues/218>`_
* Contributors: Jihoon Lee, Matt Vollrath, Russell Toris

0.7.14 (2016-02-11)
-------------------
* Abort websocket server listen() retry on shutdown
  This allows the server to shut down via SIGINT or SIGTERM during its listen() retry loop.
* rospy.get_param instead of get_param
* actually use those parameters
* remove reference to retry_startup_delay from rosbridge_udp.launch
* clean up parameters and handling
  * make parameters accessible via parameter server for all three versions
  * remove old advertise_service parameters
  * UDP and TCP can't do SSL
  * TCP can't authenticate yet (because the RosbridgeTcpSocket class is instantiated for each request and hence does not hold state)
  * UDP does not take a hostname or address, but rather an interface
* Allow TCP Server to reuse address after restart
  After killing (Ctrl-C) a rosbridge_tcp server instance which has
  connected clients, starting a new instance (on the same port) can
  fail with the error: '[Errno 98] Address already in use'. Although the
  node retries until the server starts, this can take up to a few minutes.
  Instruct the ThreadingTCPServer to allow the reuse of the same address.
* Adding UDP
* Contributors: Matt Vollrath, Nils Berg, Victor Savu, XuHao, xuhao1

0.7.13 (2015-08-14)
-------------------
* Add bson encoding to the server side
* Add default strings for certfile and keyfile
  This allows downstream packages with roslaunch_add_file_check tests to pass.
* Fix whitespace in RosbridgeTcpHandler
* Modularize RosbridgeTcpSocket
* Modularize RosbridgeWebSocket
* add shutdown handling to rosbridge_tcp and make rosbridge_websocket more robust
* Removed space from empty line.
  Thanks @T045T
* Stop IOLoop on shutdown.
* Contributors: Benny, David Lu, Matt Vollrath, Nils Berg, Paul Bovbel

0.7.12 (2015-04-07)
-------------------

0.7.11 (2015-03-23)
-------------------
* rename rosapi script to rosapi_node to address `#170 <https://github.com/RobotWebTools/rosbridge_suite/issues/170>`_
* Enabled TCP nodelay in Websocket handler
* Contributors: Jihoon Lee, Sebastien Mamessier

0.7.10 (2015-02-25)
-------------------

0.7.9 (2015-02-24)
------------------

0.7.8 (2015-01-16)
------------------
* Fix path to Tornado speedup extension source
* Build Tornado speedups
  Fixes `#135 <https://github.com/RobotWebTools/rosbridge_suite/issues/135>`_
* Contributors: Matt Vollrath

0.7.7 (2015-01-06)
------------------
* remove rosbridge_tools from dependency `#163 <https://github.com/RobotWebTools/rosbridge_suite/issues/163>`_
* reverting back the changes
* Contributors: Jihoon Lee

0.7.6 (2014-12-26)
------------------
* 0.7.5
* update changelog
* Function in robridge_tools for importing tornado
* Revert "reverts back to internal tornado until fix is ready"
  This reverts commit 49eeb1d97da154213d3170c95169b5677b329d07.
* 0.7.4
* changelog updated
* reverts back to internal tornado until fix is ready
* 0.7.3
* changelog updated
* 0.7.2
* changelog updated
* use alias to import rosbridge_tool tornado
* move modules under rosbridge_tools
* 0.7.1
* update changelog
* Merge pull request #147 from RobotWebTools/migrate_third_parties
  separate tornado and backports from rosbridge_server
* seprate out third party library and ros related script
* remove setup.py
* add rosbridge_tools as rosbridge_server dependency
* remove python-imaging dependency. it is used in rosbridge_library
* 0.7.0
* changelog updated
* Contributors: Jihoon Lee, Jon Binney, Russell Toris

0.7.5 (2014-12-26)
------------------
* Function in robridge_tools for importing tornado
* Revert "reverts back to internal tornado until fix is ready"
  This reverts commit 49eeb1d97da154213d3170c95169b5677b329d07.
* Contributors: Jon Binney

0.7.4 (2014-12-16)
------------------
* reverts back to internal tornado until fix is ready
* Contributors: Russell Toris

0.7.3 (2014-12-15)
------------------

0.7.2 (2014-12-15)
------------------
* use alias to import rosbridge_tool tornado
* move modules under rosbridge_tools
* 0.7.1
* update changelog
* Merge pull request #147 from RobotWebTools/migrate_third_parties
  separate tornado and backports from rosbridge_server
* seprate out third party library and ros related script
* remove setup.py
* add rosbridge_tools as rosbridge_server dependency
* remove python-imaging dependency. it is used in rosbridge_library
* Contributors: Jihoon Lee, Russell Toris

0.7.1 (2014-12-09)
------------------
* Merge pull request `#147 <https://github.com/RobotWebTools/rosbridge_suite/issues/147>`_ from RobotWebTools/migrate_third_parties
  separate tornado and backports from rosbridge_server
* seprate out third party library and ros related script
* remove setup.py
* add rosbridge_tools as rosbridge_server dependency
* remove python-imaging dependency. it is used in rosbridge_library
* Contributors: Jihoon Lee, Russell Toris

0.7.0 (2014-12-02)
------------------

0.6.8 (2014-11-05)
------------------

0.6.7 (2014-10-22)
------------------
* updated package manifests
* Merge pull request #137 from RobotWebTools/revert
  Revert "Install Tornado via rosdep"
* Revert "Install Tornado via rosdep"
  This reverts commit 2d8a2fa5d23550427d6957acffc7dfa6f55e9c34.
* Contributors: Russell Toris

0.6.6 (2014-10-21)
------------------
* Install Tornado via rosdep
  Use python-tornado-pip to make sure we get the speedups introduced in Tornado 3.2.
* Contributors: Matt Vollrath

0.6.5 (2014-10-14)
------------------
* 0.6.4
* update changelog
* add backports to setup.py, so backports.ssl_match_hostname can be properly resolved
* 0.6.3
* update change log
* Contributors: Jihoon Lee, Nils Berg

0.6.4 (2014-10-08)
------------------
* add backports to setup.py, so backports.ssl_match_hostname can be properly resolved
* Contributors: Nils Berg

0.6.3 (2014-10-07)
------------------

0.6.2 (2014-10-06)
------------------
* Merge pull request #125 from megawac/json
  Remove unused imports; move json imports to utility
* override to enable support for allowing alternate origins
  To accept all cross-origin traffic (which was the default prior to Tornado 4.0), simply override this method to always return true.
* import backports.ssl_match_hostname 3.4.0.2
* upgrade tornado to 4.0.2
* Remove unused json imports; move json imports to utility
  Fixes #7
* Contributors: Graeme Yeates, Ramon Wijnands, Russell Toris

0.6.1 (2014-09-01)
------------------

0.6.0 (2014-05-23)
------------------

0.5.4 (2014-04-17)
------------------

0.5.3 (2014-03-28)
------------------
* rosbridge_server: add install tag for python files, not just symlinks, to make them executable
* Contributors: ipa-mig

0.5.2 (2014-03-14)
------------------
* move global param into local param to address issue `#25 <https://github.com/RobotWebTools/rosbridge_suite/issues/25>`_
* moving global parameter into local parameter to address issue `#25 <https://github.com/RobotWebTools/rosbridge_suite/issues/25>`_
* merging changes of groovy-devel into hydro-devel
* Specific IP adress binding using roslauch
* added parameter lookup to rosbridge_tcp.py, modules where those are used, and default parameters to launch file; internal default-values still get used when launch-file does not provide them; internal defaults can be changed within rosbridge_tcp.py
* increaing max_msg_length - still hardcoded
* preparing pull request for upstream..
* cleanup: files, notes, some code
* cleanup tcp-server
* added message_field <message_intervall> to allow client to control delay between messages from rosbridge
* tested rosbridge_websocket with new capabilities; websocket test scripts not working yet..; but new caps are working when using rosbridge_websocket and tcp2ws wrapper --> so only testscripts need to be fixed for websockets.
* feierabend.. morgen weiter mit server & client JSON-decoder, see notes
* only current changes; not yet done..
* code cleanup, not yet finished..; rosbridge logging much cleaner now
* file extension for websocket server .py
* ...
* ...
* fixed test_server_defragment - recodegit status
* linuxonandroid
* added extension to server script; + symlink
* fixed some parts; ..still better do some redesign for queueing of messages..
* forced tcp_send to use queue and use delay between sends
* blocking behavior for service requests to non-ros; test-scripts use get-ip4 helper function; ..needs a lot cleanup before next steps..
* message_size debugging; TODO: sort list of received fragments! ; make sure receive_buffers are big enough for fragment_size + header..
* some code cleanup
* some minor changes: comments, debug-output, ..
* first working classes: service_server
* added socket_timeout and exception-handling for clients that do not send any data at all but are listening only.
* Catkin fixes for rosbridge TCP.
* Catkinizes rosbridge_tcp.
  Adds launch file too.
* Clean up of Rosbridge TCP.
* add rosbridge_server with tcp socket support
* adapt rosbridge_tcp to groovy-devel structure
* add rosbridge_server with tcp socket support
* Param bug fixed
* SSL options added
* Contributors: Brandon Alexander, Jihoon Lee, Russell Toris, Steffel Fenix, dave, fxm-db, ipa-fxm, root

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
* launch file location fixed in install
* response from rosauth fixed
* authentication added
* launch file updated with args for port and SSL options
* SSL options added
* eclipse projects removed
* Contributors: Russell Toris

0.4.1 (2013-03-07)
------------------

0.4.0 (2013-03-05)
------------------
* Resolves submodule issues.
* Adds rosbridge_websocket launch file.
* Uses only 1 .gitignore to avoid confusion.
* Fixing rosapi's "Cannot include proxy..." errors.
* Renames server script to rosbridge_websocket.
* Adds BSD license header to code files.
  See Issue `#13 <https://github.com/RobotWebTools/rosbridge_suite/issues/13>`_.
* rosbridge_server requires rosapi.
* Moves rosbridge_server code to scripts.
  Was getting an odd bug with tornado:
  [ERROR] [WallTime: 1356115083.100585] Uncaught exception, closing connection.
  [ERROR] [WallTime: 1356115083.100900] Exception in callback <tornado.stack_context._StackContextWrapper object at 0x1dd6e10>
* Removing ultrajson from rosbridge.
  If JSON parsing becomes a performance bottle neck, we can readd it.
* Refactors rosbridge_server. Adds scripts dir.
* Catkinizing rosbridge_library and server.
* Added command line --port argument.
* Collapse directory structure.
* Moved the packages inside a folder called rosbridge
* Initial commit of rosbridge_server
* Contributors: Austin Hendrix, Brandon Alexander, Jonathan Mace
