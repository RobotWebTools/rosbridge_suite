cmake_minimum_required(VERSION 2.8.3)
project(test_rosbridge_library)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS message_generation rospy rosbridge_library roscpp rosgraph rostest)

#######################################
## Declare ROS messages and services ##
#######################################

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  CharTest.msg
  DurationArrayTest.msg
  HeaderArrayTest.msg
  HeaderTest.msg
  HeaderTestTwo.msg
  TimeArrayTest.msg
  UInt8Test.msg
)

# Generate services in the 'srv' folder
add_service_files(
  FILES
  ArrayRequest.srv
  Empty.srv
  MultipleRequestFields.srv
  MultipleResponseFields.srv
  RequestAndResponse.srv
  RequestOnly.srv
  ResponseOnly.srv
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################################
## Declare things to be passed to other projects ##
###################################################

catkin_package()

#############
## Testing ##
#############

add_rostest(test/capabilities/test_capabilities.test)
add_rostest(test/internal/test_internal.test)

