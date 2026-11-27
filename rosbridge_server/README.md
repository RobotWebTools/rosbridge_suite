## About

Rosbridge server is part of the rosbridge_suite of packages, providing a
WebSocket transport layer. A
[WebSocket](http://www.html5rocks.com/en/tutorials/websockets/basics/)
is a low-latency, bidirectional communication layer between clients (web
browsers) and servers. By providing a WebSocket connection, rosbridge
server allows webpages to talk ROS using the rosbridge protocol.

Rosbridge server creates a WebSocket connection and passes any JSON
messages from the WebSocket to rosbridge_library, so rosbridge library
can convert the JSON strings into ROS calls. The reverse also happens,
with rosbridge library converting any ROS responses into JSON, then
passing it to rosbridge server to send over the WebSocket connection.

Check out [roslibjs](roslibjs) for a JavaScript library that talks to
rosbridge server over WebSockets.

## Other Transport Layers

While rosbridge server provides a WebSocket connection, rosbridge itself
is not tied to any particular transport layer. You could add a rosbridge
TCP package or node, for example, that communicates with rosbridge using
TCP sockets. No WebSockets or browsers needed.
[Pull requests](https://github.com/RobotWebTools/rosbridge_suite) for
these additional transport layers are encouraged!
