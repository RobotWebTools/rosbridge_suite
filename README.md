rosbridge_suite
===============

[![ROS Humble version](https://img.shields.io/ros/v/humble/rosbridge_suite)](https://index.ros.org/p/rosbridge_suite/github-RobotWebTools-rosbridge_suite/#humble)
[![ROS Jazzy version](https://img.shields.io/ros/v/jazzy/rosbridge_suite)](https://index.ros.org/p/rosbridge_suite/github-RobotWebTools-rosbridge_suite/#jazzy)
[![ROS Rolling version](https://img.shields.io/ros/v/rolling/rosbridge_suite)](https://index.ros.org/p/rosbridge_suite/github-RobotWebTools-rosbridge_suite/#rolling)


#### Server Implementations of the rosbridge v2 Protocol

rosbridge provides a JSON interface to ROS, allowing any client to send JSON to publish or subscribe to ROS topics, call ROS services, and more. rosbridge supports a variety of transport layers, including WebSockets and TCP. For information on the protocol itself, see the [rosbridge protocol specification](ROSBRIDGE_PROTOCOL.md).

For full documentation, see [the ROS wiki](http://ros.org/wiki/rosbridge_suite).

This project is released as part of the [Robot Web Tools](https://robotwebtools.github.io/) effort.

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
 * [roslibrust](https://github.com/Carter12s/roslibrust) - A Rust API, which communicates with rosbridge over WebSockets.

### License
rosbridge_suite is released with a BSD license. For full terms and conditions, see the [LICENSE](LICENSE) file.

### Authors
See the [AUTHORS](AUTHORS.md) file for a full list of contributors.

### Quality Declaration
This package claims to be in the **Quality Level 3** category, see the [Quality Declaration](/QUALITY_DECLARATION.md) for more details.

### Troubleshooting

See the [TROUBLESHOOTING](TROUBLESHOOTING.md) doc for common problems and solutions.

### Release process

Releasing requires push access to [RobotWebTools/rosbridge_suite](https://github.com/RobotWebTools/rosbridge_suite) as well as [ros2-gbp/rosbridge_suite-release](https://github.com/ros2-gbp/rosbridge_suite-release). For more details on how the release process works, see the [bloom tutorial](https://docs.ros.org/en/galactic/How-To-Guides/Releasing-a-ROS-2-package-with-bloom.html).

1. Run `catkin_generate_changelog` to update CHANGELOG.rst files.
2. Manually edit and clean up the changelogs. Commit the changes.
3. Run `catkin_prepare_release --bump [major/minor/patch]` to bump versions in package.xml and push changes to origin.
4. Run bloom-release commands to create PRs to update rosdistro:
    - `bloom-release --rosdistro humble rosbridge_suite`
    - `bloom-release --rosdistro jazzy rosbridge_suite`
    - `bloom-release --rosdistro rolling rosbridge_suite`

Note that right now, the Humble release is tracked in the `humble` branch, while Jazzy and later are tracked in the `ros2` branch.

Once the PRs are merged, packages will be available for each distro after the next sync.
Build/sync status can be viewed at: [humble](http://repo.ros2.org/status_page/ros_humble_default.html), [jazzy](http://repo.ros2.org/status_page/ros_jazzy_default.html), and [rolling](http://repo.ros2.org/status_page/ros_rolling_default.html).
