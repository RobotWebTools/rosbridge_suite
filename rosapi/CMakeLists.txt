cmake_minimum_required(VERSION 3.5)
project(rosapi)

find_package(ament_cmake_ros REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

ament_python_install_package(
  ${PROJECT_NAME} PACKAGE_DIR "src/${PROJECT_NAME}")

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/TypeDef.msg
  srv/DeleteParam.srv
  srv/GetActionServers.srv
  srv/GetParam.srv
  srv/GetParamNames.srv
  srv/GetTime.srv
  srv/HasParam.srv
  srv/MessageDetails.srv
  srv/Nodes.srv
  srv/NodeDetails.srv
  srv/Publishers.srv
  srv/SearchParam.srv
  srv/ServiceHost.srv
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
  srv/TopicsForType.srv
  srv/TopicType.srv
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()

install(PROGRAMS scripts/rosapi_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES mapping_rules.yaml
  DESTINATION share/${PROJECT_NAME}
)
