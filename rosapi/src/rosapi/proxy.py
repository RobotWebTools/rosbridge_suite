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

from ros2action.api import get_action_names_and_types
from ros2interface.api import type_completer
from ros2node.api import (
    get_node_names,
    get_publisher_info,
    get_service_server_info,
    get_subscriber_info,
)
from ros2service.api import get_service_names, get_service_names_and_types
from ros2topic.api import get_topic_names, get_topic_names_and_types

from .glob_helper import any_match, filter_globs

_node = None


def init(node):
    """
    Initializes proxy module with a rclpy.node.Node for further use.
    This function has to be called before any other for the module to work.
    """
    global _node
    _node = node


def get_topics(topics_glob, include_hidden=False):
    """Returns a list of all the active topics in the ROS system"""
    topic_names = get_topic_names(node=_node, include_hidden_topics=include_hidden)
    return filter_globs(topics_glob, topic_names)


def get_interfaces():
    """Returns a list of all the types in the ROS system"""
    return type_completer()


def get_topics_and_types(topics_glob, include_hidden=False):
    return get_publications_and_types(
        topics_glob, get_topic_names_and_types, include_hidden_topics=include_hidden
    )


def get_topics_for_type(topic_type, topics_glob, include_hidden=False):
    topic_names_and_types = get_topic_names_and_types(
        node=_node, include_hidden_topics=include_hidden
    )
    # topic[0] has the topic name and topic[1] has the type wrapped in a list.
    topics_for_type = [topic[0] for topic in topic_names_and_types if topic[1][0] == topic_type]
    return filter_globs(topics_glob, topics_for_type)


def get_services(services_glob, include_hidden=False):
    """Returns a list of all the services advertised in the ROS system"""
    # Filter the list of services by whether they are public before returning.
    service_names = get_service_names(node=_node, include_hidden_services=include_hidden)
    return filter_globs(services_glob, service_names)


def get_services_and_types(services_glob, include_hidden=False):
    return get_publications_and_types(
        services_glob,
        get_service_names_and_types,
        include_hidden_services=include_hidden,
    )


def get_services_for_type(service_type, services_glob, include_hidden=False):
    """Returns a list of services as specific service type"""
    # Filter the list of services by whether they are public before returning.
    services_names_and_types = get_service_names_and_types(
        node=_node, include_hidden_services=include_hidden
    )
    # service[0] has the topic name and service[1] has the type wrapped in a list.
    services_for_type = [
        service[0] for service in services_names_and_types if service[1][0] == service_type
    ]
    return filter_globs(services_glob, services_for_type)


def get_publications_and_types(glob, getter_function, **include_hidden_publications):
    """Generic getter function for both services and topics"""
    publication_names_and_types = getter_function(node=_node, **include_hidden_publications)
    # publication[0] has the publication name and publication[1] has the type wrapped in a list.
    all_publications = [publication[0] for publication in publication_names_and_types]
    filtered_publications = filter_globs(glob, all_publications)
    filtered_publication_types = [
        publication[1][0]
        for publication in publication_names_and_types
        if publication[0] in filtered_publications
    ]
    return filtered_publications, filtered_publication_types


def get_nodes(include_hidden=False):
    """Returns a list of all the nodes registered in the ROS system"""
    node_names = get_node_names(node=_node, include_hidden_nodes=include_hidden)
    full_names = [node_name.full_name for node_name in node_names]
    return full_names


def get_node_info(node_name, include_hidden=False):
    node_names = get_node_names(node=_node, include_hidden_nodes=include_hidden)
    if node_name in [n.full_name for n in node_names]:
        # Only the name of each item is required as output.
        subscribers = get_node_subscriptions(node_name)
        publishers = get_node_publications(node_name)
        services = get_node_services(node_name)

        return subscribers, publishers, services


def get_node_publications(node_name):
    """Returns a list of topic names that are being published by the specified node"""
    publishers = get_publisher_info(node=_node, remote_node_name=node_name)
    return [publisher.name for publisher in publishers]


def get_node_subscriptions(node_name):
    """Returns a list of topic names that are being subscribed by the specified node"""
    subscribers = get_subscriber_info(node=_node, remote_node_name=node_name)
    return [subscriber.name for subscriber in subscribers]


def get_node_services(node_name):
    """Returns a list of service names that are being hosted by the specified node"""
    services = get_service_server_info(node=_node, remote_node_name=node_name)
    return [service.name for service in services]


def get_node_service_types(node_name):
    """Returns a list of service types that are being hosted by the specified node"""
    services = get_service_server_info(node=_node, remote_node_name=node_name)
    return [service.types[0] for service in services]


def get_topic_type(topic, topics_glob):
    """Returns the type of the specified ROS topic"""
    # Note: this doesn't consider hidden topics.
    topics, types = get_topics_and_types(topics_glob)
    try:
        return types[topics.index(topic)]
    except ValueError:
        # Return empty string if the topic is not present.
        return ""


def filter_action_servers(topics):
    """Returns a list of action servers"""
    # Note(@jubeira): filtering by topic should be enough; services can be taken into account as well.
    action_servers = []
    possible_action_server = ""
    possibility = [0, 0]

    action_topics = ["feedback", "status"]
    for topic in sorted(topics):
        split = topic.split("/")
        if len(split) >= 4:
            topic = split.pop()
            action_prefix = split.pop()
            if action_prefix != "_action":
                continue

            namespace = "/".join(split)
            if possible_action_server != namespace:
                possible_action_server = namespace
                possibility = [0, 0]
            if possible_action_server == namespace and topic in action_topics:
                possibility[action_topics.index(topic)] = 1
            if all(p == 1 for p in possibility):
                action_servers.append(possible_action_server)
                possibility = [0, 0]

    return action_servers


def get_service_type(service, services_glob):
    """Returns the type of the specified ROS service,"""
    # Note: this doesn't consider hidden services.
    services, types = get_services_and_types(services_glob)
    try:
        return types[services.index(service)]
    except ValueError:
        # Return empty string if the service is not present.
        return ""


def get_channel_info(channel, channels_glob, getter_function, include_hidden=False):
    """Returns a list of node names that are publishing / subscribing to the specified topic,
    or advertising a given service."""
    if any_match(str(channel), channels_glob):
        channel_info_list = []
        node_list = get_nodes(include_hidden)
        for node in node_list:
            channel_info = getter_function(node)
            if channel in channel_info:
                channel_info_list.append(node)
        return channel_info_list
    else:
        return []


def get_publishers(topic, topics_glob, include_hidden=False):
    """Returns a list of node names that are publishing the specified topic"""
    return get_channel_info(
        topic, topics_glob, get_node_publications, include_hidden=include_hidden
    )


def get_subscribers(topic, topics_glob, include_hidden=False):
    """Returns a list of node names that are subscribing to the specified topic"""
    return get_channel_info(
        topic, topics_glob, get_node_subscriptions, include_hidden=include_hidden
    )


def get_service_providers(queried_type, services_glob, include_hidden=False):
    """Returns a list of node names that are advertising a service with the specified type"""
    return get_channel_info(
        queried_type,
        services_glob,
        get_node_service_types,
        include_hidden=include_hidden,
    )


def get_service_node(queried_type, services_glob, include_hidden=False):
    """Returns the name of the node that is providing the given service, or empty string"""
    node_name = get_channel_info(
        queried_type, services_glob, get_node_services, include_hidden=include_hidden
    )
    if node_name:
        return node_name[0]
    else:
        return ""


def get_action_type(action_name, include_hidden=False):
    """Returns the type of the specified ROS action.
    If the action does not exist, an empty string is returned."""

    names_and_types = get_action_names_and_types(node=_node, include_hidden_actions=include_hidden)

    for name, types in names_and_types:
        if name == action_name and types:
            return types[0]

    return ""
