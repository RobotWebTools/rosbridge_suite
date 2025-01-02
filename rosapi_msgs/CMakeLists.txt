cmake_minimum_required(VERSION 3.5)
project(rosapi_msgs)

find_package(ament_cmake_ros REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/TypeDef.msg
  srv/DeleteParam.srv
  srv/GetActionServers.srv
  srv/GetParam.srv
  srv/GetParamNames.srv
  srv/GetROSVersion.srv
  srv/GetTime.srv
  srv/HasParam.srv
  srv/Interfaces.srv
  srv/MessageDetails.srv
  srv/Nodes.srv
  srv/NodeDetails.srv
  srv/Publishers.srv
  srv/ServiceNode.srv
  srv/ServiceProviders.srv
  srv/ServiceRequestDetails.srv
  srv/ServiceResponseDetails.srv
  srv/Services.srv
  srv/ServicesForType.srv
  srv/ServiceType.srv
  srv/SetParam.srv
  srv/Subscribers.srv
  srv/Topics.srv
  srv/TopicsAndRawTypes.srv
  srv/TopicsForType.srv
  srv/TopicType.srv
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(builtin_interfaces rosidl_default_runtime)

ament_package()
