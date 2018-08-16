cmake_minimum_required(VERSION 3.5)
project(rosbridge_server NONE)

find_package(ament_cmake_core REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(
  ${PROJECT_NAME} PACKAGE_DIR "src/${PROJECT_NAME}")

ament_package()

install(PROGRAMS
  scripts/rosbridge_websocket.py
  scripts/rosbridge_websocket
  scripts/rosbridge_tcp.py
  scripts/rosbridge_tcp
  scripts/rosbridge_udp
  scripts/rosbridge_udp.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  launch/rosbridge_websocket.launch
  launch/rosbridge_tcp.launch
  launch/rosbridge_udp.launch
  DESTINATION share/${PROJECT_NAME}/launch
)

