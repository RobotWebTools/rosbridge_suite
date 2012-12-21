cmake_minimum_required(VERSION 2.8.3)
project(rosapi)

find_package(catkin REQUIRED COMPONENTS rosbridge_library rospy)

catkin_package()
catkin_python_setup()

install(PROGRAMS scripts/rosapi
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

