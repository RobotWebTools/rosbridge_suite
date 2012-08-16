#!/usr/bin/env python
from roslib import load_manifest; load_manifest('rosapi');
from rospy import init_node
from rospy import spin
from rospy import Service
from rospy import ROSInterruptException
from rospy import loginfo

import proxy, objectutils, params
from rosapi.srv import *
from rosapi.msg import *

# Initialises the ROS node
def register_services():
    Service('rosapi/topics', Topics, get_topics)
    Service('rosapi/topics_for_type', TopicsForType, get_topics_for_type)
    Service('rosapi/services', Services, get_services)
    Service('rosapi/nodes', Nodes, get_nodes)
    Service('rosapi/topic_type', TopicType, get_topic_type)
    Service('rosapi/service_type', ServiceType, get_service_type)
    Service('rosapi/publishers', Publishers, get_publishers)
    Service('rosapi/subscribers', Subscribers, get_subscribers)
    Service('rosapi/service_providers', ServiceProviders, get_service_providers)
    Service('rosapi/service_node', ServiceNode, get_service_node)
    Service('rosapi/service_host', ServiceHost, get_service_host)
    Service('rosapi/message_details', MessageDetails, get_message_details)
    Service('rosapi/service_request_details', ServiceRequestDetails, get_service_request_details)
    Service('rosapi/service_response_details', ServiceResponseDetails, get_service_response_details)
    Service('rosapi/set_param', SetParam, set_param)
    Service('rosapi/get_param', GetParam, get_param)
    Service('rosapi/has_param', HasParam, has_param)
    Service('rosapi/search_param', SearchParam, search_param)
    Service('rosapi/delete_param', DeleteParam, delete_param)
    
def get_topics(request):
    """ Called by the rosapi/Topics service. Returns a list of all the topics being published. """
    return TopicsResponse(proxy.get_topics())

def get_topics_for_type(request):
    """ Called by the rosapi/TopicsForType service. Returns a list of all the topics that are publishing a given type """
    return TopicsForTypeResponse(proxy.get_topics_for_type(request.type))

def get_services(request):
    """ Called by the rosapi/Services service. Returns a list of all the services being advertised. """
    return ServicesResponse(proxy.get_services())

def get_nodes(request):
    """ Called by the rosapi/Nodes service. Returns a list of all the nodes that are registered """
    return NodesResponse(proxy.get_nodes())
    
def get_topic_type(request):
    """ Called by the rosapi/TopicType service.  Given the name of a topic, returns the name of the type of that topic.
    Request class has one field, 'topic', which is a string value (the name of the topic)
    Response class has one field, 'type', which is a string value (the type of the topic)
    If the topic does not exist, an empty string is returned. """
    return TopicTypeResponse(proxy.get_topic_type(request.topic))
    
def get_service_type(request):
    """ Called by the rosapi/ServiceType service.  Given the name of a service, returns the type of that service
    Request class has one field, 'service', which is a string value (the name of the service)
    Response class has one field, 'type', which is a string value (the type of the service)
    If the service does not exist, an empty string is returned. """
    return ServiceTypeResponse(proxy.get_service_type(request.service))

def get_publishers(request):
    """ Called by the rosapi/Publishers service.  Given the name of a topic, returns a list of node names
    that are publishing on that topic. """
    return PublishersResponse(proxy.get_publishers(request.topic))

def get_subscribers(request):
    """ Called by the rosapi/Subscribers service.  Given the name of a topic, returns a list of node names
    that are subscribing to that topic. """
    return SubscribersResponse(proxy.get_subscribers(request.topic))

def get_service_providers(request):
    """ Called by the rosapi/ServiceProviders service.  Given the name of a topic, returns a list of node names
    that are advertising that service type """
    return ServiceProvidersResponse(proxy.get_service_providers(request.service))

def get_service_node(request):
    """ Called by the rosapi/ServiceNode service.  Given the name of a service, returns the name of the node
    that is providing that service. """
    return ServiceNodeResponse(proxy.get_service_node(request.service))

def get_service_host(request):
    """ Called by the rosapi/ServiceNode service.  Given the name of a service, returns the name of the machine
    that is hosting that service. """
    return ServiceHostResponse(proxy.get_service_host(request.service))

def get_message_details(request):
    """ Called by the rosapi/MessageDetails service.  Given the name of a message type, returns the TypeDef
    for that type."""
    return MessageDetailsResponse([dict_to_typedef(d) for d in objectutils.get_typedef_recursive(request.type)])

def get_service_request_details(request):
    """ Called by the rosapi/ServiceRequestDetails service. Given the name of a service type, returns the TypeDef
    for the request message of that service type. """
    return ServiceRequestDetailsResponse([dict_to_typedef(d) for d in objectutils.get_service_request_typedef_recursive(request.type)])

def get_service_response_details(request):
    """ Called by the rosapi/ServiceResponseDetails service. Given the name of a service type, returns the TypeDef
    for the response message of that service type. """
    return ServiceResponseDetailsResponse([dict_to_typedef(d) for d in objectutils.get_service_response_typedef_recursive(request.type)])

def set_param(request):
    params.set_param(request.name, request.value)
    return SetParamResponse()

def get_param(request):
    return GetParamResponse(params.get_param(request.name, request.default))

def has_param(request):
    return HasParamResponse(params.has_param(request.name))

def search_param(request):
    return SearchParamResponse(params.search_param(request.name))

def delete_param(request):
    params.delete_param(request.name)
    return DeleteParamResponse()

def dict_to_typedef(typedefdict):
    typedef = TypeDef()
    typedef.type = typedefdict["type"]
    typedef.fieldnames = typedefdict["fieldnames"]
    typedef.fieldtypes = typedefdict["fieldtypes"]
    typedef.fieldarraylen = typedefdict["fieldarraylen"]
    typedef.examples = typedefdict["examples"]
    return typedef

if __name__ == '__main__':
     try:
         init_node('rosapi')
         register_services()
         loginfo("Rosapi started")
         spin()
     except ROSInterruptException: 
         pass
