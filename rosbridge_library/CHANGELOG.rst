^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosbridge_library
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.5 (2014-03-28)
------------------
* update changelog
* adding missing dependency in rosbridge_library `#70 <https://github.com/RobotWebTools/rosbridge_suite/issues/70>`_
* support publishing non-ascii letters
* added parameter lookup to rosbridge_tcp.py, modules where those are used, and default parameters to launch file; internal default-values still get used when launch-file does not provide them; internal defaults can be changed within rosbridge_tcp.py
* Merge branch 'experimental_branch' into new_features
* moved tests to ...test/experimental/...
* fix handling of partial/multiple/broken json by avoiding to pass nested json (without op-field) to rosbridge.. probably still needs more complex handling of incoming 'broken' json
* nested service not MiRoR related anymore
* fixed minor things..
* added singleton for request-list; allows provider to send service response without specifying module and type, they get looked up when response is received via request_id
* fix for nested service responses - use ros_loader and message_conversion for populating an according instance
* use message_conversion in handle_servie_request
* snapshot for branch to show to genpy devs
* using float64 instead of std_msgs/Float64 lets scripts run fine.. ; next: fix with using std_msgs/Float64 --> need nested data field
* nested srv uses now message_conversion.extract_values
* adapted test scripted to ros_loader; (removed .srv from module_name
* use rosloader for finding service_class
* fixed calculation of fragment_count
* preparing for pull request to upstream
* preparing pull request for upstream..
* preparing pull request for upstream..
* preparing pull request for upstream..
* cleanup
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
* even more missing depends for unit tests
* more missing test packages
* missing depends added when running tests
* rostest now uses devel instead of install
* rostest added to package
* commented out that problematic unregister line
* Contributors: Brandon Alexander, Jihoon Lee, Kaijen Hsiao, Russell Toris, dave, furushchev, fxm-db, ipa-fxm, root, unknown

* adding missing dependency in rosbridge_library `#70 <https://github.com/RobotWebTools/rosbridge_suite/issues/70>`_
* support publishing non-ascii letters
* added parameter lookup to rosbridge_tcp.py, modules where those are used, and default parameters to launch file; internal default-values still get used when launch-file does not provide them; internal defaults can be changed within rosbridge_tcp.py
* Merge branch 'experimental_branch' into new_features
* moved tests to ...test/experimental/...
* fix handling of partial/multiple/broken json by avoiding to pass nested json (without op-field) to rosbridge.. probably still needs more complex handling of incoming 'broken' json
* nested service not MiRoR related anymore
* fixed minor things..
* added singleton for request-list; allows provider to send service response without specifying module and type, they get looked up when response is received via request_id
* fix for nested service responses - use ros_loader and message_conversion for populating an according instance
* use message_conversion in handle_servie_request
* snapshot for branch to show to genpy devs
* using float64 instead of std_msgs/Float64 lets scripts run fine.. ; next: fix with using std_msgs/Float64 --> need nested data field
* nested srv uses now message_conversion.extract_values
* adapted test scripted to ros_loader; (removed .srv from module_name
* use rosloader for finding service_class
* fixed calculation of fragment_count
* preparing for pull request to upstream
* added message_field <message_intervall> to allow client to control delay between messages from rosbridge
* added TODO: check if service successfully registered in ros
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
* removed obsolete test-scripts
* stop service added
* first working classes: service_server
* should use its own branch: service_server.py;  add initial thoughts and code-base for developing ServiceServer capability
* fixed errors in protocol.py and defragmentation.py
* added test-scripts for defragmentation AND tcp-server
* change json imports to try to use ujson or simplejson; correct log_message to show length of content/data instead of overall length
* fixed variable name in finish()
* Clean up of defragmentation.py.
* add defragmentation capability
* merge with fuerte-devel
* add defragmentation capability
* even more missing depends for unit tests
* more missing test packages
* missing depends added when running tests
* rostest now uses devel instead of install
* rostest added to package
* commented out that problematic unregister line
* Contributors: Brandon Alexander, Jihoon Lee, Kaijen Hsiao, Russell Toris, dave, furushchev, fxm-db, ipa-fxm, root, unknown
