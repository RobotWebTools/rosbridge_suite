Rosbridge provides a JSON interface to ROS, allowing any client to send JSON to
publish or subscribe to ROS topics, call ROS services, and more. Rosbridge
supports a variety of transport layers, including WebSockets.

### Packages

 * rosbridge_suite is a [ROS
   meta-package](http://www.ros.org/wiki/catkin/conceptual_overview#Metapackages_and_the_Elimination_of_Stacks),
   simply including all the rosbridge packages.

 * rosbridge_library contains the Python API that receives JSON-formatted
   strings as input and controls ROS publishers/subscribers/service calls
   according to the content of the JSON strings.

 * rosbridge_server contains a WebSocket server implementation that exposes the
   rosbridge_library.

 * rosapi provides service calls for getting meta-information related to ROS
   like topic lists as well as interacting with the Parameter Server.

### Clients

A rosbridge client is a program that communicates with rosbridge using its JSON
API. Rosbridge clients include:

 * [ros.js](http://www.ros.org/wiki/rosjs) - A JavaScript API, which
   communicates with rosbridge over WebSockets.

### Resources

 * [Documentation](http://www.ros.org/wiki/rosbridge_suite)
 * [Coding Style Guide](http://www.ros.org/wiki/PyStyleGuide)
 * [Contributors](https://github.com/RobotWebTools/rosbridge_suite/graphs/contributors)
 * [License (BSD)](http://opensource.org/licenses/BSD-2-Clause)

### Change Logs

 * 0.4.3
  * launch file location fixed
  * SSL option added
  * Authentication added
