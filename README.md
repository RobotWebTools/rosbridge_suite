Rosbridge provides a JSON interface to ROS, allowing any client to send JSON to
publish or subscribe to ROS topics, call ROS services, and more. Rosbridge
supports a variety of transport layers, including WebSockets.

Rosbridge packages include:

 * rosbridge_library contains the Python API that receives JSON-formatted
   strings as input and controls ROS publishers/subscribers/service calls
   according to the content of the JSON strings.

 * rosbridge_server contains a WebSocket server implementation that exposes the
   rosbridge_library.

A rosbridge client is a program that communicates with rosbridge using its JSON
API. Rosbridge clients include:

 * [ros.js](http://www.ros.org/wiki/rosjs) - A JavaScript API, which
   communicates with rosbridge over WebSockets.

Further resources:

 * [Documentation](http://www.ros.org/wiki/rosbridge_suite)
 * [Coding Style Guide](http://www.ros.org/wiki/PyStyleGuide)
 * [Contributors](https://github.com/RobotWebTools/rosbridge_suite/graphs/contributors)
 * [License (BSD)](http://opensource.org/licenses/BSD-2-Clause)

