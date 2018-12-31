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

import fnmatch
import socket
from rosservice import get_service_list
from rosservice import get_service_type as rosservice_get_service_type
from rosservice import get_service_node as rosservice_get_service_node
from rosservice import get_service_uri
from rosservice import rosservice_find
from rostopic import find_by_type
from rostopic import get_topic_type as rosservice_get_topic_type
from rosnode import get_node_names
from rosgraph.masterapi import Master

from rosapi.msg import TypeDef

from .glob_helper import filter_globs, any_match


def get_topics_and_types(topics_glob):
    """ Returns a list of all the active topics in the ROS system """
    try:
        # Function getTopicTypes also returns inactive topics and does not
        # return topics with unknown message types, so it must be compared
        # to results from getSystemState.
        master = Master('/rosbridge')
        topic_types = master.getTopicTypes()
        publishers, subscribers, services = master.getSystemState()
        topics = set([x for x, _ in publishers] + [x for x, _ in subscribers])

        # Filter the list of topics by whether they are public.
        topics = set(filter_globs(topics_glob, topics))
        topic_types = [x for x in topic_types if x[0] in topics]

        # Add topics with unknown type messages.
        unknown_type = topics.difference([x for x, _ in topic_types])
        return zip(* topic_types + [[x,''] for x in unknown_type])
    except:
        return []


def get_topics_for_type(type, topics_glob):
    # Filter the list of topics by whether they are public before returning.
    return filter_globs(topics_glob, find_by_type(type))


def get_services(services_glob):
    """ Returns a list of all the services advertised in the ROS system """
    # Filter the list of services by whether they are public before returning.
    return filter_globs(services_glob, get_service_list())


def get_services_for_type(service_type, services_glob):
    """ Returns a list of services as specific service type """
    # Filter the list of services by whether they are public before returning.
    return filter_globs(services_glob, rosservice_find(service_type))


def get_nodes():
    """ Returns a list of all the nodes registered in the ROS system """
    return get_node_names()


def get_node_publications(node):
    """ Returns a list of topic names that are been published by the specified node """
    try:
        publishers, subscribers, services = Master('/rosbridge').getSystemState()
        toReturn = []
        for i, v in publishers:
            if node in v:
                toReturn.append(i)
        toReturn.sort()
        return toReturn
    except socket.error:
        return []


def get_node_subscriptions(node):
    """ Returns a list of topic names that are been subscribed by the specified node """
    try:
        publishers, subscribers, services = Master('/rosbridge').getSystemState()
        toReturn = []
        for i, v in subscribers:
            if node in v:
                toReturn.append(i)
        toReturn.sort()
        return toReturn
    except socket.error:
        return []


def get_node_services(node):
    """ Returns a list of service names that are been hosted by the specified node """
    try:
        publishers, subscribers, services = Master('/rosbridge').getSystemState()
        toReturn = []
        for i, v in services:
            if node in v:
                toReturn.append(i)
        toReturn.sort()
        return toReturn
    except socket.error:
        return []


def get_topic_type(topic, topics_glob):
    """ Returns the type of the specified ROS topic """
    # Check if the topic is hidden or public.
    # If all topics are public then the type is returned
    if any_match(str(topic), topics_glob):
        # If the topic is published, return its type
        topic_type, _, _ = rosservice_get_topic_type(topic)
        if topic_type is None:
            # Topic isn't published so return an empty string
            return ""
        return topic_type
    else:
        # Topic is hidden so return an empty string
        return ""


def filter_action_servers(topics):
    """ Returns a list of action servers """
    action_servers = []
    possible_action_server = ''
    possibility = [0, 0, 0, 0, 0]

    action_topics = ['cancel', 'feedback', 'goal', 'result', 'status']
    for topic in sorted(topics):
        split = topic.split('/')
        if(len(split) >= 3):
            topic = split.pop()
            namespace = '/'.join(split)
            if(possible_action_server != namespace):
                possible_action_server = namespace
                possibility = [0, 0, 0, 0, 0]
            if possible_action_server == namespace and topic in action_topics:
                possibility[action_topics.index(topic)] = 1
        if all(p == 1 for p in possibility):
            action_servers.append(possible_action_server)

    return action_servers


def get_service_type(service, services_glob):
    """ Returns the type of the specified ROS service, """
    # Check if the service is hidden or public.
    if any_match(str(service), services_glob):
        try:
            return rosservice_get_service_type(service)
        except:
            return ""
    else:
        # Service is hidden so return an empty string.
        return ""


def get_publishers(topic, topics_glob):
    """ Returns a list of node names that are publishing the specified topic """
    try:
        if any_match(str(topic), topics_glob):
            publishers, subscribers, services = Master('/rosbridge').getSystemState()
            pubdict = dict(publishers)
            if topic in pubdict:
                return pubdict[topic]
            else:
                return []
        else:
            return []
    except socket.error:
        return []


def get_subscribers(topic, topics_glob):
    """ Returns a list of node names that are subscribing to the specified topic """
    try:
        if any_match(str(topic), topics_glob):
            publishers, subscribers, services = Master('/rosbridge').getSystemState()
            subdict = dict(subscribers)
            if topic in subdict:
                return subdict[topic]
            else:
                return []
        else:
            return []
    except socket.error:
        return []


def get_service_providers(queried_type, services_glob):
    """ Returns a list of node names that are advertising a service with the specified type """
    _, _, services = Master('/rosbridge').getSystemState()

    service_type_providers = []
    for service, providers in services:
        service_type = get_service_type(service, services_glob)

        if service_type == queried_type:
            service_type_providers += providers
    return service_type_providers


def get_service_node(service):
    """ Returns the name of the node that is providing the given service, or empty string """
    node = rosservice_get_service_node(service)
    if node == None:
        node = ""
    return node


def get_service_host(service):
    """ Returns the name of the machine that is hosting the given service, or empty string """
    uri = get_service_uri(service)
    if uri == None:
        uri = ""
    else:
        uri = uri[9:]
        uri = uri[:uri.find(':')]
    return uri
