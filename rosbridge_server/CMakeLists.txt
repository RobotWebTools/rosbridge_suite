cmake_minimum_required(VERSION 3.5)
project(rosbridge_server)

find_package(ament_cmake REQUIRED)
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

  # Run each test with both SingleThreadedExecutor and EventsExecutor
  set(TEST_FILES
    test/websocket/advertise_action.test.py
    test/websocket/advertise_action_feedback.test.py
    test/websocket/advertise_service.test.py
    test/websocket/call_service.test.py
    test/websocket/send_action_goal.test.py
    test/websocket/smoke.test.py
    test/websocket/transient_local_publisher.test.py
    test/websocket/best_effort_publisher.test.py
    test/websocket/multiple_subscribers_raw.test.py
  )

  foreach(TEST_FILE ${TEST_FILES})
    # Extract base name for test naming
    get_filename_component(TEST_NAME ${TEST_FILE} NAME_WE)

    # Run with SingleThreadedExecutor (default)
    add_launch_test(${TEST_FILE}
      TARGET ${TEST_NAME}_singlethreaded
      ARGS "use_events_executor:=false"
    )

    # Run with EventsExecutor
    add_launch_test(${TEST_FILE}
      TARGET ${TEST_NAME}_events
      ARGS "use_events_executor:=true"
    )
  endforeach()

  find_package(ament_cmake_mypy REQUIRED)
  ament_mypy()
endif()
