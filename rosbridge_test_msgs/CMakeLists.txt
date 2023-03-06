cmake_minimum_required(VERSION 3.5)
project(rosbridge_test_msgs)

find_package(ament_cmake_core REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/Num.msg
  msg/TestChar.msg
  msg/TestDurationArray.msg
  msg/TestHeaderArray.msg
  msg/TestHeader.msg
  msg/TestHeaderTwo.msg
  msg/TestTimeArray.msg
  msg/TestUInt8.msg
  msg/TestUInt8FixedSizeArray16.msg
  msg/TestFloat32Array.msg
  msg/TestFloat32BoundedArray.msg
  msg/TestNestedBoundedArray.msg
  srv/AddTwoInts.srv
  srv/SendBytes.srv
  srv/TestArrayRequest.srv
  srv/TestEmpty.srv
  srv/TestMultipleRequestFields.srv
  srv/TestMultipleResponseFields.srv
  srv/TestNestedService.srv
  srv/TestRequestAndResponse.srv
  srv/TestRequestOnly.srv
  srv/TestResponseOnly.srv
  action/TestEmpty.action
  action/TestFeedbackAndResult.action
  action/TestGoalAndResult.action
  action/TestGoalFeedbackAndResult.action
  action/TestGoalOnly.action
  action/TestMultipleGoalFields.action
  action/TestResultOnly.action
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
