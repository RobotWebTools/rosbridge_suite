cmake_minimum_required(VERSION 2.8.3)
project(rosbridge_server)

find_package(catkin REQUIRED COMPONENTS rosbridge_library rosapi rospy)

catkin_python_setup()

catkin_package()

install(PROGRAMS scripts/rosbridge_server
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

