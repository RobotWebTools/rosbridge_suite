^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosbridge_server
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* Contributors: Brandon Alexander, Jihoon Lee, Russell Toris, Steffel Fénix, dave, fxm-db, ipa-fxm, root

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
