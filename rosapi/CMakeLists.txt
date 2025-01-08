cmake_minimum_required(VERSION 3.5)
project(rosapi)

find_package(ament_cmake_ros REQUIRED)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

ament_python_install_package(
  ${PROJECT_NAME} PACKAGE_DIR "src/${PROJECT_NAME}")

ament_package()

install(PROGRAMS scripts/rosapi_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES mapping_rules.yaml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(${PROJECT_NAME}_test_stringify_field_types test/test_stringify_field_types.py)
  ament_add_pytest_test(${PROJECT_NAME}_test_typedefs test/test_typedefs.py)

  find_package(ament_cmake_mypy REQUIRED)
  ament_mypy()
endif()
