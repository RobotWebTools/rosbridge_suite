#!/usr/bin/env python3
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

import os
import sys

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node

from rosapi import glob_helper, objectutils, params, proxy
from rosapi_msgs.msg import TypeDef
from rosapi_msgs.srv import (
    DeleteParam,
    GetActionServers,
    GetParam,
    GetParamNames,
    GetROSVersion,
    GetTime,
    HasParam,
    MessageDetails,
    NodeDetails,
    Nodes,
    Publishers,
    ServiceNode,
    ServiceProviders,
    ServiceRequestDetails,
    ServiceResponseDetails,
    Services,
    ServicesForType,
    ServiceType,
    SetParam,
    Subscribers,
    Topics,
    TopicsAndRawTypes,
    TopicsForType,
    TopicType,
)


class Rosapi(Node):

    NAME = "rosapi"

    def __init__(self):
        super().__init__(self.NAME)
        self.declare_parameter("topics_glob", "[*]")
        self.declare_parameter("services_glob", "[*]")
        self.declare_parameter("params_glob", "[*]")
        self.globs = self.get_globs()
        self.register_services()

    # Initialises the ROS node
    def register_services(self):
        proxy.init(self)
        if self.get_namespace() == "/":
            full_name = self.get_namespace() + self.get_name()
        else:
            full_name = self.get_namespace() + "/" + self.get_name()
        params.init(full_name)
        self.create_service(Topics, "/rosapi/topics", self.get_topics)
        self.create_service(TopicsForType, "/rosapi/topics_for_type", self.get_topics_for_type)
        self.create_service(
            TopicsAndRawTypes,
            "/rosapi/topics_and_raw_types",
            self.get_topics_and_raw_types,
        )
        self.create_service(Services, "/rosapi/services", self.get_services)
        self.create_service(
            ServicesForType, "/rosapi/services_for_type", self.get_services_for_type
        )
        self.create_service(Nodes, "/rosapi/nodes", self.get_nodes)
        self.create_service(NodeDetails, "/rosapi/node_details", self.get_node_details)
        self.create_service(GetActionServers, "/rosapi/action_servers", self.get_action_servers)
        self.create_service(TopicType, "/rosapi/topic_type", self.get_topic_type)
        self.create_service(ServiceType, "/rosapi/service_type", self.get_service_type)
        self.create_service(Publishers, "/rosapi/publishers", self.get_publishers)
        self.create_service(Subscribers, "/rosapi/subscribers", self.get_subscribers)
        self.create_service(
            ServiceProviders, "/rosapi/service_providers", self.get_service_providers
        )
        self.create_service(ServiceNode, "/rosapi/service_node", self.get_service_node)
        self.create_service(MessageDetails, "/rosapi/message_details", self.get_message_details)
        self.create_service(
            ServiceRequestDetails,
            "/rosapi/service_request_details",
            self.get_service_request_details,
        )
        self.create_service(
            ServiceResponseDetails,
            "/rosapi/service_response_details",
            self.get_service_response_details,
        )
        self.create_service(SetParam, "/rosapi/set_param", self.set_param)
        self.create_service(GetParam, "/rosapi/get_param", self.get_param)
        self.create_service(HasParam, "/rosapi/has_param", self.has_param)
        self.create_service(DeleteParam, "/rosapi/delete_param", self.delete_param)
        self.create_service(GetParamNames, "/rosapi/get_param_names", self.get_param_names)
        self.create_service(GetTime, "/rosapi/get_time", self.get_time)
        self.create_service(GetROSVersion, "/rosapi/get_ros_version", self.get_ros_version)

    def get_globs(self):
        return glob_helper.get_globs(self)

    def get_topics(self, request, response):
        """Called by the rosapi/Topics service. Returns a list of all the topics being published."""
        response.topics, response.types = proxy.get_topics_and_types(self.globs.topics)
        return response

    def get_topics_for_type(self, request, response):
        """Called by the rosapi/TopicsForType service. Returns a list of all the topics that are publishing a given type"""
        response.topics = proxy.get_topics_for_type(request.type, self.globs.topics)
        return response

    def get_topics_and_raw_types(self, request, response):
        """Called by the rosapi/TopicsAndRawTypes service. Returns a list of all the topics being published, and their
        raw types, similar to `gendeps --cat`."""
        response.topics, response.types = proxy.get_topics_and_types(self.globs.topics)
        response.typedefs_full_text = [
            objectutils.get_typedef_full_text(type) for type in response.types
        ]
        return response

    def get_services(self, request, response):
        """Called by the rosapi/Services service. Returns a list of all the services being advertised."""
        response.services = proxy.get_services(self.globs.services)
        return response

    def get_services_for_type(self, request, response):
        """Called by the rosapi/ServicesForType service. Returns a list of all the services that are publishing a given type"""
        response.services = proxy.get_services_for_type(request.type, self.globs.services)
        return response

    def get_nodes(self, request, response):
        """Called by the rosapi/Nodes service. Returns a list of all the nodes that are registered"""
        response.nodes = proxy.get_nodes()
        return response

    def get_node_details(self, request, response):
        """Called by the rosapi/Nodes service. Returns a node description"""
        (
            response.subscribing,
            response.publishing,
            response.services,
        ) = proxy.get_node_info(request.node)
        return response

    def get_action_servers(self, request, response):
        """Called by the rosapi/GetActionServers service. Returns a list of action servers based on actions standard topics"""
        topics = proxy.get_topics(self.globs.topics, include_hidden=True)
        response.action_servers = proxy.filter_action_servers(topics)
        return response

    def get_topic_type(self, request, response):
        """Called by the rosapi/TopicType service.  Given the name of a topic, returns the name of the type of that topic.
        Request class has one field, 'topic', which is a string value (the name of the topic)
        Response class has one field, 'type', which is a string value (the type of the topic)
        If the topic does not exist, an empty string is returned."""
        response.type = proxy.get_topic_type(request.topic, self.globs.topics)
        return response

    def get_service_type(self, request, response):
        """Called by the rosapi/ServiceType service.  Given the name of a service, returns the type of that service
        Request class has one field, 'service', which is a string value (the name of the service)
        Response class has one field, 'type', which is a string value (the type of the service)
        If the service does not exist, an empty string is returned."""
        response.type = proxy.get_service_type(request.service, self.globs.services)
        return response

    def get_publishers(self, request, response):
        """Called by the rosapi/Publishers service.  Given the name of a topic, returns a list of node names
        that are publishing on that topic."""
        response.publishers = proxy.get_publishers(request.topic, self.globs.topics)
        return response

    def get_subscribers(self, request, response):
        """Called by the rosapi/Subscribers service.  Given the name of a topic, returns a list of node names
        that are subscribing to that topic."""
        response.subscribers = proxy.get_subscribers(request.topic, self.globs.topics)
        return response

    def get_service_providers(self, request, response):
        """Called by the rosapi/ServiceProviders service.  Given the name of a topic, returns a list of node names
        that are advertising that service type"""
        response.providers = proxy.get_service_providers(request.service, self.globs.services)
        return response

    def get_service_node(self, request, response):
        """Called by the rosapi/ServiceNode service.  Given the name of a service, returns the name of the node
        that is providing that service."""
        response.node = proxy.get_service_node(request.service, self.globs.services)
        return response

    def get_message_details(self, request, response):
        """Called by the rosapi/MessageDetails service.  Given the name of a message type, returns the TypeDef
        for that type."""
        response.typedefs = [
            dict_to_typedef(d) for d in objectutils.get_typedef_recursive(request.type)
        ]
        return response

    def get_service_request_details(self, request, response):
        """Called by the rosapi/ServiceRequestDetails service. Given the name of a service type, returns the TypeDef
        for the request message of that service type."""
        response.typedefs = [
            dict_to_typedef(d)
            for d in objectutils.get_service_request_typedef_recursive(request.type)
        ]
        return response

    def get_service_response_details(self, request, response):
        """Called by the rosapi/ServiceResponseDetails service. Given the name of a service type, returns the TypeDef
        for the response message of that service type."""
        response.typedefs = [
            dict_to_typedef(d)
            for d in objectutils.get_service_response_typedef_recursive(request.type)
        ]
        return response

    def set_param(self, request, response):
        try:
            node_name, param_name = self._get_node_and_param_name(request.name)
            params.set_param(node_name, param_name, request.value, self.globs.params)
        except ValueError:
            self._print_malformed_param_name_warning(request.name)
        return response

    def get_param(self, request, response):
        try:
            node_name, param_name = self._get_node_and_param_name(request.name)
            response.value = params.get_param(
                node_name, param_name, request.default_value, self.globs.params
            )
        except ValueError:
            self._print_malformed_param_name_warning(request.name)
        return response

    def has_param(self, request, response):
        try:
            node_name, param_name = self._get_node_and_param_name(request.name)
            response.exists = params.has_param(node_name, param_name, self.globs.params)
        except ValueError:
            self._print_malformed_param_name_warning(request.name)
        return response

    def delete_param(self, request, response):
        params.delete_param(request.node_name, request.name, self.globs.params)
        return response

    def get_param_names(self, request, response):
        response.names = params.get_param_names(self.globs.params)
        return response

    def get_time(self, request, response):
        response.time = Clock(clock_type=ClockType.ROS_TIME).now().to_msg()
        return response

    def _get_node_and_param_name(self, param):
        return tuple(param.split(":"))

    def _print_malformed_param_name_warning(self, param_name):
        self.get_logger().warn(
            "Malformed parameter name: {}; expecting <node_name>:<param_name>".format(param_name)
        )

    def get_ros_version(self, request, response):
        response.version = 2
        response.distro = str(os.environ["ROS_DISTRO"])
        return response


def dict_to_typedef(typedefdict):
    typedef = TypeDef()
    typedef.type = typedefdict["type"]
    typedef.fieldnames = typedefdict["fieldnames"]
    typedef.fieldtypes = typedefdict["fieldtypes"]
    typedef.fieldarraylen = typedefdict["fieldarraylen"]
    typedef.examples = typedefdict["examples"]
    typedef.constnames = typedefdict["constnames"]
    typedef.constvalues = typedefdict["constvalues"]
    return typedef


def main(args=None):
    if args is None:
        args = sys.argv

    rclpy.init(args=args)
    node = Rosapi()
    try:
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        print("Exiting due to SIGINT")


if __name__ == "__main__":
    main()
