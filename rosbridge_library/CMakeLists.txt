cmake_minimum_required(VERSION 2.8.3)
project(rosbridge_library)

find_package(catkin REQUIRED COMPONENTS message_generation rospy rosservice rostopic roscpp rosgraph geometry_msgs)

# Generate messages for testing (NOINSTALL)
add_message_files(
  FILES
  Num.msg
  TestChar.msg
  TestDurationArray.msg
  TestHeaderArray.msg
  TestHeader.msg
  TestHeaderTwo.msg
  TestTimeArray.msg
  TestUInt8.msg
  TestUInt8FixedSizeArray16.msg
  NOINSTALL
)

# Generate services for testing (NOINSTALL)
add_service_files(
  FILES
  AddTwoInts.srv
  SendBytes.srv
  TestArrayRequest.srv
  TestEmpty.srv
  TestMultipleRequestFields.srv
  TestMultipleResponseFields.srv
  TestNestedService.srv  
  TestRequestAndResponse.srv
  TestRequestOnly.srv
  TestResponseOnly.srv
  NOINSTALL
)

catkin_python_setup()

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package()

# Test launch files
# add_rostest(test/test_all.test)

