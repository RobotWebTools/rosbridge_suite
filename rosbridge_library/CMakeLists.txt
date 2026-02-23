cmake_minimum_required(VERSION 3.5)
project(rosbridge_library)

find_package(ament_cmake_ros REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(
  ${PROJECT_NAME} PACKAGE_DIR "src/${PROJECT_NAME}")

ament_package()

if (BUILD_TESTING)
  ament_add_ros_isolated_pytest_test(test_capabilities "test/capabilities/")
  ament_add_ros_isolated_pytest_test(test_internal "test/internal/")

  find_package(ament_cmake_mypy REQUIRED)
  ament_mypy()
endif()
