cmake_minimum_required(VERSION 2.8.3)
project(rosapi)

find_package(catkin REQUIRED COMPONENTS rospy message_generation)
catkin_python_setup()

add_message_files(
  FILES
  TypeDef.msg
)

add_service_files(
  FILES
  DeleteParam.srv
  GetActionServers.srv
  GetParam.srv
  GetParamNames.srv
  GetTime.srv
  HasParam.srv
  MessageDetails.srv
  Nodes.srv
  NodeDetails.srv
  Publishers.srv
  SearchParam.srv
  ServiceHost.srv
  ServiceNode.srv
  ServiceProviders.srv
  ServiceRequestDetails.srv
  ServiceResponseDetails.srv
  Services.srv
  ServicesForType.srv
  ServiceType.srv
  SetParam.srv
  Subscribers.srv
  Topics.srv
  TopicsForType.srv
  TopicType.srv
)

generate_messages(
  DEPENDENCIES
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

install(PROGRAMS scripts/rosapi_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
