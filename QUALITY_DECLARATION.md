This document is a declaration of software quality for the `rosbridge_suite` packages, based on the guidelines in [REP-2004](https://www.ros.org/reps/rep-2004.html). The following packages are covered by this declaration:

- rosapi
- rosapi_msgs
- rosbridge_library
- rosbridge_msgs
- rosbridge_server
- rosbridge_suite
- rosbridge_test_msgs

# `rosbridge_suite` Quality Declaration

The package `rosbridge_suite` claims to be in the **Quality Level 3** category.

Below are the rationales, notes, and caveats for this claim, organized by each requirement listed in the [Package Requirements for Quality Level N in REP-2004](https://www.ros.org/reps/rep-2004.html).

## Version Policy [1]

### Version Scheme [1.i]

`rosbridge_suite` uses [semver](https://semver.org/) according to the recommendation in the [ROS 2 Developer Guide](https://docs.ros.org/en/rolling/Contributing/Developer-Guide.html#versioning).

### Version Stability [1.ii]

`rosbridge_suite` is at a stable version, i.e. `>= 1.0.0`. The current version can be found in its [package.xml](/rosbridge_server/package.xml), and its change history can be found in its [CHANGELOG](/rosbridge_server/CHANGELOG.rst).

### Public API Declaration [1.iii]

The public API is not explicitly defined.

### API Stability Within a Released ROS Distribution [1.iv]/[1.vi]

The public API is not guaranteed to be stable within a released ROS distribution.

### ABI Stability Within a Released ROS Distribution [1.v]/[1.vi]

`rosbridge_suite` is written in Python and therefore is not concerned with ABI stability.

## Change Control Process [2]

`rosbridge_suite` follows the recommended guidelines for ROS Core packages in the [ROS 2 Developer Guide](https://docs.ros.org/en/rolling/Contributing/Developer-Guide.html#change-control-process).

### Change Requests [2.i]

All changes will occur through a pull request, check [ROS 2 Developer Guide](https://index.ros.org/doc/ros2/Contributing/Developer-Guide/#change-control-process) for additional information.

### Continuous Integration [2.ii]

All pull request must pass CI on all supported platforms.

## Documentation [3]

### License [3.i]

The license for `rosbridge_suite` is BSD 3-clause, and a full copy is in the [LICENSE](/LICENSE) file.

### Copyright Statement [3.ii]

The copyright statement is in the [LICENSE](/LICENSE) file.

## Testing [4]

New features are required to have tests before being added.

## Dependencies [5]


`rosbridge_suite` has a runtime dependency on `rclpy`, which does not have a quality declaration.

## Platform Support [6]

`rosbridge_suite` supports all of the tier 1 platforms as described in [REP-2000](https://www.ros.org/reps/rep-2000.html#support-tiers). Tests are currently only run against Ubuntu Linux.

## Security [7]

### Vulnerability Disclosure Policy [7.i]

This package conforms to the Vulnerability Disclosure Policy in [REP-2006](https://www.ros.org/reps/rep-2006.html).
