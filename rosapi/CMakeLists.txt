cmake_minimum_required(VERSION 2.8.3)
project(rosapi)

find_package(catkin REQUIRED COMPONENTS rosbridge_library roscpp rospy std_msgs message_generation)
catkin_python_setup()

add_message_files(
  FILES
  TypeDef.msg
)

add_service_files(
  FILES
  DeleteParam.srv
  GetParam.srv
  GetParamNames.srv
  GetTime.srv
  HasParam.srv
  MessageDetails.srv
  Nodes.srv
  Publishers.srv
  SearchParam.srv
  ServiceHost.srv
  ServiceNode.srv
  ServiceProviders.srv
  ServiceRequestDetails.srv
  ServiceResponseDetails.srv
  Services.srv
  ServiceType.srv
  SetParam.srv
  Subscribers.srv
  Topics.srv
  TopicsForType.srv
  TopicType.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

install(PROGRAMS scripts/rosapi
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

