cmake_minimum_required(VERSION 3.5)
project(rosbridge_server)

find_package(ament_cmake_ros REQUIRED)
find_package(ament_cmake_core REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(
  ${PROJECT_NAME} PACKAGE_DIR "src/${PROJECT_NAME}")

ament_package()

install(PROGRAMS
  scripts/rosbridge_websocket.py
  scripts/rosbridge_websocket
  scripts/rosbridge_tcp
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/websocket/advertise_service.test.py)
  add_launch_test(test/websocket/call_service.test.py)
  add_launch_test(test/websocket/smoke.test.py)
  add_launch_test(test/websocket/transient_local_publisher.test.py)
  add_launch_test(test/websocket/best_effort_publisher.test.py)
  add_launch_test(test/websocket/multiple_subscribers_raw.test.py)
endif()
