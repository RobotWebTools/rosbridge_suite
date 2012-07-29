#!/usr/bin/env python
import roslib; roslib.load_manifest('rosapi');
import rospy

import proxy, objectutils
from rosapi.srv import *
from rosapi.msg import *

# Initialises the ROS node
def registerServices():
    rospy.Service('rosapi/Topics', Topics, getTopics)
    rospy.Service('rosapi/TopicsForType', TopicsForType, getTopicsForType)
    rospy.Service('rosapi/Services', Services, getServices)
    rospy.Service('rosapi/Nodes', Nodes, getNodes)
    rospy.Service('rosapi/TopicType', TopicType, getTopicType)
    rospy.Service('rosapi/ServiceType', ServiceType, getServiceType)
    rospy.Service('rosapi/Publishers', Publishers, getPublishers)
    rospy.Service('rosapi/Subscribers', Subscribers, getSubscribers)
    rospy.Service('rosapi/ServiceProviders', ServiceProviders, getServiceProviders)
    rospy.Service('rosapi/ServiceNode', ServiceNode, getServiceNode)
    rospy.Service('rosapi/ServiceHost', ServiceHost, getServiceHost)
    rospy.Service('rosapi/MessageDetails', MessageDetails, getMessageDetails)
    rospy.Service('rosapi/ServiceRequestDetails', ServiceRequestDetails, getServiceRequestDetails)
    rospy.Service('rosapi/ServiceResponseDetails', ServiceResponseDetails, getServiceResponseDetails)
    
def getTopics(request):
    """ Called by the rosapi/Topics service. Returns a list of all the topics being published. """
    return TopicsResponse(proxy.getTopics())

def getTopicsForType(request):
    """ Called by the rosapi/TopicsForType service. Returns a list of all the topics that are publishing a given type """
    return TopicsForTypeResponse(proxy.getTopicsForType(request.type))

def getServices(request):
    """ Called by the rosapi/Services service. Returns a list of all the services being advertised. """
    return ServicesResponse(proxy.getServices())

def getNodes(request):
    """ Called by the rosapi/Nodes service. Returns a list of all the nodes that are registered """
    return NodesResponse(proxy.getNodes())
    
def getTopicType(request):
    """ Called by the rosapi/TopicType service.  Given the name of a topic, returns the name of the type of that topic.
    Request class has one field, 'topic', which is a string value (the name of the topic)
    Response class has one field, 'type', which is a string value (the type of the topic)
    If the topic does not exist, an empty string is returned. """
    return TopicTypeResponse(proxy.getTopicType(request.topic))
    
def getServiceType(request):
    """ Called by the rosapi/ServiceType service.  Given the name of a service, returns the type of that service
    Request class has one field, 'service', which is a string value (the name of the service)
    Response class has one field, 'type', which is a string value (the type of the service)
    If the service does not exist, an empty string is returned. """
    return ServiceTypeResponse(proxy.getServiceType(request.service))

def getPublishers(request):
    """ Called by the rosapi/Publishers service.  Given the name of a topic, returns a list of node names
    that are publishing on that topic. """
    return PublishersResponse(proxy.getPublishers(request.topic))

def getSubscribers(request):
    """ Called by the rosapi/Subscribers service.  Given the name of a topic, returns a list of node names
    that are subscribing to that topic. """
    return SubscribersResponse(proxy.getSubscribers(request.topic))

def getServiceProviders(request):
    """ Called by the rosapi/ServiceProviders service.  Given the name of a topic, returns a list of node names
    that are advertising that service type """
    return ServiceProvidersResponse(proxy.getServiceProviders(request.service))

def getServiceNode(request):
    """ Called by the rosapi/ServiceNode service.  Given the name of a service, returns the name of the node
    that is providing that service. """
    return ServiceNodeResponse(proxy.getServiceNode(request.service))

def getServiceHost(request):
    """ Called by the rosapi/ServiceNode service.  Given the name of a service, returns the name of the machine
    that is hosting that service. """
    return ServiceHostResponse(proxy.getServiceHost(request.service))

def getMessageDetails(request):
    """ Called by the rosapi/MessageDetails service.  Given the name of a message type, returns the TypeDef
    for that type."""
    return MessageDetailsResponse([_toTypeDefMsg(d) for d in objectutils.getTypeDefRecursive(request.type)])

def getServiceRequestDetails(request):
    """ Called by the rosapi/ServiceRequestDetails service. Given the name of a service type, returns the TypeDef
    for the request message of that service type. """
    return ServiceRequestDetailsResponse([_toTypeDefMsg(d) for d in objectutils.getServiceRequestTypeDefRecursive(request.type)])

def getServiceResponseDetails(request):
    """ Called by the rosapi/ServiceResponseDetails service. Given the name of a service type, returns the TypeDef
    for the response message of that service type. """
    return ServiceResponseDetailsResponse([_toTypeDefMsg(d) for d in objectutils.getServiceResponseTypeDefRecursive(request.type)])

def _toTypeDefMsg(typedefdict):
    typedef = TypeDef()
    typedef.type = typedefdict["type"]
    typedef.fieldnames = typedefdict["fieldnames"]
    typedef.fieldtypes = typedefdict["fieldtypes"]
    typedef.fieldarraylen = typedefdict["fieldarraylen"]
    typedef.examples = typedefdict["examples"]
    return typedef

if __name__ == '__main__':
     try:
         rospy.init_node('rosapi')
         registerServices()
         rospy.spin()
     except rospy.ROSInterruptException: 
         pass
