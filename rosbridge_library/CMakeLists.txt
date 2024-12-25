cmake_minimum_required(VERSION 3.5)
project(rosbridge_library)

find_package(ament_cmake_ros REQUIRED)
find_package(ament_cmake_core REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(
  ${PROJECT_NAME} PACKAGE_DIR "src/${PROJECT_NAME}")

ament_package()

if (BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_capabilities "test/capabilities/")
  ament_add_pytest_test(test_internal "test/internal/")

  find_package(ament_cmake_mypy REQUIRED)
  ament_mypy()
endif()
