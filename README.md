rosbridge_suite [![Build Status](https://api.travis-ci.org/RobotWebTools/rosbridge_suite.png)](https://travis-ci.org/RobotWebTools/rosbridge_suite)
===============

#### Server Implementations of the rosbridge v2 Protocol

rosbridge provides a JSON interface to ROS, allowing any client to send JSON to publish or subscribe to ROS topics, call ROS services, and more. rosbridge supports a variety of transport layers, including WebSockets and TCP. For information on the protocol itself, see the [rosbridge protocol specification](ROSBRIDGE_PROTOCOL.md).

For full documentation, see [the ROS wiki](http://ros.org/wiki/rosbridge_suite).

This project is released as part of the [Robot Web Tools](http://robotwebtools.org/) effort.

### Packages

 * [rosbridge_suite](rosbridge_suite) is a [ROS meta-package](http://www.ros.org/wiki/catkin/conceptual_overview#Metapackages_and_the_Elimination_of_Stacks) including all the rosbridge packages.

 * [rosbridge_library](rosbridge_library) contains the Python API that receives JSON-formatted strings as input and controls ROS publishers/subscribers/service calls according to the content of the JSON strings.

 * [rosbridge_server](rosbridge_server) contains a WebSocket server implementation that exposes the rosbridge_library.

 * [rosapi](rosapi) provides service calls for getting meta-information related to ROS like topic lists as well as interacting with the Parameter Server.

### Clients

A rosbridge client is a program that communicates with rosbridge using its JSON API. rosbridge clients include:

 * [roslibjs](https://github.com/RobotWebTools/roslibjs) - A JavaScript API, which communicates with rosbridge over WebSockets.
 * [jrosbridge](https://github.com/WPI-RAIL/jrosbridge) - A Java API, which communicates with rosbridge over WebSockets.
 * [roslibpy](https://github.com/gramaziokohler/roslibpy) - A Python API, which communicates with rosbridge over WebSockets.

### Development

You can use the `run_docker.sh` script to get an interactive shell in a container with rosbridge in a catkin workspace. The sources will be mounted read-only in the container, so any changes you make outside the container will be reflected inside. Host networking will be used so you can run interact with ROS nodes and access the server from outside the container. You can use this to quickly iterate without having to rebuild the image for most changes.

### License
rosbridge_suite is released with a BSD license. For full terms and conditions, see the [LICENSE](LICENSE) file.

### Authors
See the [AUTHORS](AUTHORS.md) file for a full list of contributors.
