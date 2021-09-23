cmake_minimum_required(VERSION 3.5)
project(rosbridge_library)

find_package(ament_cmake_core REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

ament_python_install_package(
  ${PROJECT_NAME} PACKAGE_DIR "src/${PROJECT_NAME}")

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
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()

if (BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  # TODO: Enable tests
  # ament_add_pytest_test(test_capabilities "test/capabilities")
  # ament_add_pytest_test(test_internal "test/internal")
endif()
