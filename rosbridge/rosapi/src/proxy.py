#!/usr/bin/env python
import roslib; roslib.load_manifest('rosservice'); roslib.load_manifest('rospy')
import rospy
import rosservice, rostopic
from ros import rosnode
from ros import rosgraph
from rosgraph import masterapi

from rosapi.msg import TypeDef

import re

import objectutils
import registry

#fuerte/electric decisions
importedTime = False

try:
    import roslib.rostime
    importedTime = True
except ImportError, ign:
    pass

try:
    if not importedTime:
        import rospy.rostime
        importedTime = True
except ImportError, ign:
    pass

if not importedTime:
    raise ImportError

# Keep track of which manifests and modules we have already loaded
loaded_manifests = {}
loaded_modules = {}

def getTopics():
    """ Returns a list of all the topics being published in the ROS system """
    return [x[0] for x in rospy.get_published_topics()]

def getTopicsForType(type):
    return rostopic.find_by_type(type)

def getServices():
    """ Returns a list of all the services advertised in the ROS system """
    return rosservice.get_service_list()

def getNodes():
    """ Returns a list of all the nodes registered in the ROS system """
    return rosnode.get_node_names()
    
def getTopicType(topic):
    """ Returns the type of the specified ROS topic """
    # If the topic is published, return its type
    for x in rospy.get_published_topics():
        if x[0]==topic:
            return x[1]
    # Topic isn't published so return an empty string
    return ""

def getServiceType(service):
    """ Returns the type of the specified ROS service, """
    try:
        return rosservice.get_service_type(service)
    except:
        return ""
    
def getPublishers(topic):
    """ Returns a list of node names that are publishing the specified topic """
    try:
        publishers, subscribers, services = masterapi.Master('/rosbridge').getSystemState()
        pubdict = dict(publishers)
        if topic in pubdict:
            return pubdict[topic]
        else:
            return []
    except socket.error:
        return []
    
def getSubscribers(topic):
    """ Returns a list of node names that are subscribing to the specified topic """
    try:
        publishers, subscribers, services = masterapi.Master('/rosbridge').getSystemState()
        subdict = dict(subscribers)
        if topic in subdict:
            return subdict[topic]
        else:
            return []
    except socket.error:
        return []
    
def getServiceProviders(servicetype):
    """ Returns a list of node names that are advertising a service with the specified type """
    try:
        publishers, subscribers, services = masterapi.Master('/rosbridge').getSystemState()
        servdict = dict(services)
        if servicetype in servdict:
            return servdict[servicetype]
        else:
            return []
    except socket.error:
        return []
        
        
def getServiceNode(service):
    """ Returns the name of the node that is providing the given service, or empty string """
    node = rosservice.get_service_node(service)
    if node==None:
        node = ""
    return node

def getServiceHost(service):
    """ Returns the name of the machine that is hosting the given service, or empty string """
    uri = rosservice.get_service_uri(service)
    if uri==None:
        uri = ""
    else:
        uri = uri[9:]
        uri = uri[:uri.find(':')]
    return uri
