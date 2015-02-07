#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from rospy import get_published_topics
from rosservice import get_service_list
from rosservice import get_service_type as rosservice_get_service_type
from rosservice import get_service_node as rosservice_get_service_node
from rosservice import get_service_uri
from rostopic import find_by_type
from ros import rosnode, rosgraph
from rosnode import get_node_names
from rosgraph.masterapi import Master

from rosapi.msg import TypeDef


def get_topics():
    """ Returns a list of all the topics being published in the ROS system """
    return [x[0] for x in get_published_topics()]


def get_topics_for_type(type):
    return find_by_type(type)


def get_services():
    """ Returns a list of all the services advertised in the ROS system """
    return get_service_list()


def get_nodes():
    """ Returns a list of all the nodes registered in the ROS system """
    return rosnode.get_node_names()

    
def get_topic_type(topic):
    """ Returns the type of the specified ROS topic """
    # If the topic is published, return its type
    for x in get_published_topics():
        if x[0]==topic:
            return x[1]
    # Topic isn't published so return an empty string
    return ""


def get_service_type(service):
    """ Returns the type of the specified ROS service, """
    try:
        return rosservice_get_service_type(service)
    except:
        return ""

    
def get_publishers(topic):
    """ Returns a list of node names that are publishing the specified topic """
    try:
        publishers, subscribers, services = Master('/rosbridge').getSystemState()
        pubdict = dict(publishers)
        if topic in pubdict:
            return pubdict[topic]
        else:
            return []
    except socket.error:
        return []

    
def get_subscribers(topic):
    """ Returns a list of node names that are subscribing to the specified topic """
    try:
        publishers, subscribers, services = Master('/rosbridge').getSystemState()
        subdict = dict(subscribers)
        if topic in subdict:
            return subdict[topic]
        else:
            return []
    except socket.error:
        return []

    
def get_service_providers(servicetype):
    """ Returns a list of node names that are advertising a service with the specified type """
    try:
        publishers, subscribers, services = Master('/rosbridge').getSystemState()
        servdict = dict(services)
        if servicetype in servdict:
            return servdict[servicetype]
        else:
            return []
    except socket.error:
        return []
        
        
def get_service_node(service):
    """ Returns the name of the node that is providing the given service, or empty string """
    node = rosservice_get_service_node(service)
    if node==None:
        node = ""
    return node


def get_service_host(service):
    """ Returns the name of the machine that is hosting the given service, or empty string """
    uri = get_service_uri(service)
    if uri==None:
        uri = ""
    else:
        uri = uri[9:]
        uri = uri[:uri.find(':')]
    return uri
