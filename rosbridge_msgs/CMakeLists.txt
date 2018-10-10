cmake_minimum_required(VERSION 2.8.3)
project(rosbridge_msgs)

find_package(catkin REQUIRED COMPONENTS
  genmsg std_msgs
)

add_message_files(
  FILES
  ConnectedClient.msg
  ConnectedClients.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
 CATKIN_DEPENDS std_msgs
)
