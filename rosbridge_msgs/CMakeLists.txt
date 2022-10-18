cmake_minimum_required(VERSION 3.0.2)
project(rosbridge_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation std_msgs
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
  CATKIN_DEPENDS message_runtime std_msgs
)
