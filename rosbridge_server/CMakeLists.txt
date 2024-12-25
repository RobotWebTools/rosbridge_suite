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
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  launch/rosbridge_websocket_launch.xml
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/websocket/advertise_action.test.py)
  add_launch_test(test/websocket/advertise_action_feedback.test.py)
  add_launch_test(test/websocket/advertise_service.test.py)
  add_launch_test(test/websocket/call_service.test.py)
  add_launch_test(test/websocket/send_action_goal.test.py)
  add_launch_test(test/websocket/smoke.test.py)
  add_launch_test(test/websocket/transient_local_publisher.test.py)
  add_launch_test(test/websocket/best_effort_publisher.test.py)
  add_launch_test(test/websocket/multiple_subscribers_raw.test.py)

  find_package(ament_cmake_mypy REQUIRED)
  ament_mypy()
endif()
